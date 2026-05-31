#  Industrial Machine Fault Recognition Using Audio

> **Cairo University — Faculty of Engineering | CMPS450**
> Pattern Recognition & Artificial Neural Networks

A full end-to-end pipeline for diagnosing mechanical faults in rotating industrial machinery from raw acoustic signals. The system evaluates two parallel predictive maintenance frameworks a lightweight **Edge ML** approach and a high-accuracy **Deep Learning** architecture achieving a final diagnostic accuracy of **99.75%**.

**Team:** Youssef Salem · Marwan AbdElShafy · Mohamed Ehab · Hamza Mohamed

---

##  Table of Contents

- [System Overview](#system-overview)
- [Preprocessing Pipeline](#preprocessing-pipeline)
  - [1. Data Exploration & Initial Assessment](#1-data-exploration--initial-assessment)
  - [2. Trimming & Noise Reduction](#2-trimming--noise-reduction)
  - [3. Segmentation via Sliding Windows](#3-segmentation-via-sliding-windows)
  - [4. Quality Assurance Filtering Gates](#4-quality-assurance-filtering-gates)
  - [5. Parameter Optimization & Trade-offs](#5-parameter-optimization--trade-offs)
- [Models](#models)
  - [Track A — Deep Learning (ResNet18 + CNN)](#track-a--deep-learning-resnet18--cnn)
  - [Track B — Edge Machine Learning (Random Forest)](#track-b--edge-machine-learning-random-forest)
- [Results Summary](#results-summary)
- [References](#references)

---

## System Overview

| Layer | Approach | Key Technologies |
| :--- | :--- | :--- |
| **Preprocessing** | Dynamic trimming, low-pass filtering, sliding windows | Python, librosa, SciPy |
| **Deep Learning** | Mel-spectrogram → ResNet18 CNN | PyTorch, torchaudio, torchvision |
| **Edge ML** | Statistical features → Random Forest | scikit-learn |
| **Training Env** | GPU-accelerated cloud compute | Kaggle, CUDA |

---

## Preprocessing Pipeline

The preprocessing pipeline is the backbone of this project. Raw industrial audio is inherently noisy, variable in length, and contaminated with silent phases — all of which must be resolved before any model can learn meaningful fault signatures.

The full pipeline executes six sequential stages:

```
Raw Data → Remove Offset → Low-Pass Filter → Dynamic Trimming
        → Peak Normalization → Sliding Windows → Filter Chunks → Features
```

---

### 1. Data Exploration & Initial Assessment

Before any transformation, 100 random audio samples per machine type (spanning both normal and abnormal states) were visualized in the time and frequency domains. This revealed two critical properties of the raw data:

- **Silent phases** at the start and end of recordings — artefacts of the recording setup, not machine behaviour.
- **Machine-specific frequency profiles** — each of the three machines has a distinct spectral fingerprint, with meaningful energy concentrated below 4 kHz.

These observations directly informed every subsequent design decision in the pipeline.

---

### 2. Trimming & Noise Reduction

This stage handles three separate transformations in sequence.

#### Dynamic Silence Trimming

Early iterations used a hardcoded time threshold to remove leading/trailing silence. This failed because recording durations varied widely (up to **11.0 seconds**), causing the fixed cutpoint to accidentally clip critical mechanical transients — the very events that distinguish faulty from normal operation.

The solution was a **Dynamic Distribution Analyzer**: an algorithm that trims silence relative to the signal's own peak amplitude rather than a fixed time boundary. Empirical tuning identified **−26 dB** below peak as the optimal threshold, achieving active-noise isolation with only a **5% data loss**.

#### Low-Pass Filtering

Frequency domain analysis across all three machines (averaged over 100 files each) consistently showed that energy above **4 kHz** was background noise with no diagnostic value for these specific machines. A low-pass filter was applied at 4 kHz to attenuate these high-frequency artefacts before any downstream feature extraction.

> **Note on pre-emphasis filtering:** A pre-emphasis filter (which artificially boosts high frequencies) was evaluated and explicitly rejected. While standard in speech processing, it distorted the exact frequency bands critical to mechanical fault diagnosis in this dataset.

#### Peak Normalization

After filtering, each file was amplitude-rescaled to a peak of **1.0**. This ensures that loudness differences between recordings reflect actual machine behaviour rather than inconsistencies in microphone placement or gain settings.

---

### 3. Segmentation via Sliding Windows

Both the CNN and Random Forest models require fixed-length input. Rather than cropping each file to a fixed timeframe (which risks discarding fault signatures that occur at variable times), the audio was segmented into **overlapping chunks**.

| Parameter | Value | Rationale |
| :--- | :--- | :--- |
| **Chunk size** | 0.5 seconds | Preserves short transient fault spikes |
| **Overlap** | 0.25 seconds (50%) | Ensures anomalies spanning window boundaries are captured |

The 50% overlap guarantees continuity: no mechanical event falls entirely within the discarded gap between two windows.

---

### 4. Quality Assurance Filtering Gates

Not every extracted chunk contains useful mechanical data. Chunks from ramp-up phases, sudden dropouts, or near-silent gaps would corrupt model training. Two sequential statistical gates were implemented to automatically reject low-quality segments.

#### Gate 1 — Coefficient of Variation (CV)

The **CV** (standard deviation ÷ mean) measures the energy dispersion within a chunk. A high CV indicates irregular energy behaviour — characteristic of ramp-ups, ramp-downs, or isolated impact spikes that are not representative of steady-state machine operation.

**Threshold:** Chunks with CV > **1.2** are rejected.

#### Gate 2 — Root Mean Square (RMS) Energy

Chunks that passed Gate 1 were further evaluated for absolute energy. A chunk that is statistically smooth but very quiet likely contains machine-off or ambient-noise data that bypassed the initial trimming.

**Threshold:** Chunks with RMS < **30%** of the parent file's peak RMS are rejected.

---

### 5. Parameter Optimization & Trade-offs

The final thresholds were reached through iterative empirical testing, not set a priori. The central tension was between **preprocessing recall** (retaining enough data, especially rare fault data) and **model precision** (not feeding in noise).

The primary challenge was **Machine 2**, whose fault signature consists of short, high-energy transient spikes. Initial experiments using 2.0-second chunks with 1.0-second overlap were adequate for Machines 1 and 3 (which have stable, periodic frequency profiles) but systematically eliminated Machine 2's fault chunks — the very data most critical to learn from.

The final configuration resolves this:

| Parameter | Initial | Final | Reason for Change |
| :--- | :--- | :--- | :--- |
| Chunk size | 2.0 s | **0.5 s** | Preserves short transient spikes from Machine 2 |
| CV tolerance | strict | **1.2** (relaxed) | Allows Machine 2's naturally spiky chunks through |
| RMS threshold | conservative | **0.30** (aggressive) | Aggressively drops near-silent chunks while keeping energetic ones |

This configuration preserved Machine 2's mechanical transients while maintaining a high signal-to-noise ratio across all three machines — a prerequisite for robust Edge ML deployment.

---

## Models

### Track A — Deep Learning (ResNet18 + CNN)

#### Mel-Spectrogram Transformation

Mel-spectrograms were chosen over MFCCs because CNNs learn from 2D spatial correlations. MFCCs apply a Discrete Cosine Transform that compresses the signal and discards the frequency-adjacency relationships that CNNs rely on. Mel-spectrograms preserve this spatial structure.

| Parameter | Value | Purpose |
| :--- | :--- | :--- |
| `TARGET_SR` | 16,000 Hz | Captures up to 8 kHz (Nyquist); computationally efficient |
| `N_MELS` | 128 | High-resolution frequency axis |
| `N_FFT` | 1,024 | Balances frequency detail vs. temporal blur |
| `HOP_LENGTH` | 512 | 50% overlap → smooth temporal continuity |

#### Architecture

**ResNet18** was selected as the backbone for its residual connections (mitigating vanishing gradients) and its lightweight parameter count suitable for rapid inference. Implemented in PyTorch (`torch`, `torchaudio`, `torchvision`) with GPU acceleration on Kaggle, using `num_workers=4` for continuous data streaming.

#### Handling Class Imbalance: Focal Loss

The dataset was severely imbalanced (~98.8% Normal, ~1.2% Abnormal). Standard Cross-Entropy loss was replaced with **Focal Loss**, which dynamically down-weights easy, correctly-classified examples (abundant Normal data) and focuses penalization on hard-to-classify minority examples (Abnormal faults). This prevents the model from achieving superficial accuracy by defaulting to the majority class.

#### Data Augmentation: SpecAugment

**SpecAugment** was applied exclusively to the training set. It randomly masks contiguous blocks of frequency bands and time steps, acting as structured noise injection that prevents the model from overfitting to specific background noise patterns.

- `frequency_mask = 10`, `time_mask = 20` (tuned iteratively)
- Expected artefact: training accuracy appears lower than validation accuracy during training, because training evaluates on masked (distorted) data while validation uses clean data.

#### Training Configuration

| Hyperparameter | Value |
| :--- | :--- |
| `BATCH_SIZE` | 64 |
| `LEARNING_RATE` | 0.001 |
| `EPOCHS` (max) | 30 |
| Early stopping metric | **Validation Macro F1-Score** |
| `MIN_DELTA` | 0.5% |
| `PATIENCE` | 4 epochs |

Macro F1-Score was used as the stopping criterion rather than accuracy, because accuracy is misleading on imbalanced datasets — it heavily rewards correct Normal predictions while ignoring Abnormal misses.

#### Iterative Prototyping (Toy Dataset)

A 5% subset (~11,600 chunks per class) was used to validate convergence before full training:

| Trial | Change | Key Result |
| :--- | :--- | :--- |
| Trial 1 | Accuracy as metric + aggressive SpecAugment | 50% train acc / 93.5% val acc — extreme gap |
| Trial 2 | Tuned SpecAugment down | 79.4% train acc / 97.6% val acc — healthy gap |
| Trial 3 | Macro F1 metric + Focal Loss | **Val Macro F1: 95.04%** — stable and reliable |

Trial 3 configuration was deployed to the full dataset.

#### Inference: Majority Voting

During inference, each overlapping 0.5-second chunk receives an independent prediction. These are aggregated via **majority voting**: if ≥ 50% of a file's chunks are classified as Abnormal, the entire file is flagged as faulty. Empirical threshold tuning confirmed that 50% is optimal across all machine types — no class-specific thresholds were necessary.

---

### Track B — Edge Machine Learning (Random Forest)

#### Feature Extraction

Five lightweight statistical descriptors were computed per chunk at 16 kHz:

| Feature | Domain | What It Captures |
| :--- | :--- | :--- |
| **RMS** | Time | Average signal energy / overall vibration intensity |
| **Kurtosis** | Time | Impulsiveness — high values indicate grinding or sudden impacts |
| **Crest Factor** | Time | Ratio of peak amplitude to RMS — reveals sharp acoustic spikes |
| **Skewness** | Time | Amplitude asymmetry — shifts when smooth waveforms become distorted |
| **Spectral Centroid** | Frequency | Center of mass of the spectrum — shifts as fault-related HF content emerges |

Output: `features_trackB.csv` (~1.15 million deduplicated chunks) with metadata including `chunk_filename`, `parent_file`, `machine_id`, `state`, and numeric `class_label`.

#### Classifier Configuration

A **200-tree Random Forest** was selected for its robustness to noisy features, natural non-linear decision boundaries, and interpretable feature importance scores. Train-test splitting was performed strictly at the `parent_file` level to prevent data leakage between chunks from the same recording.

#### Imbalance Mitigation

Two concurrent strategies were applied to counter the 98.8% / 1.2% imbalance:

1. **Class-weight adjustment** — Abnormal classes were assigned weights of 2.0–4.0 relative to Normal (1.0), penalising fault misclassifications more heavily.
2. **Targeted random oversampling** — Abnormal training chunks were resampled until they constituted 20–30% of the training volume. Validation and test sets were left untouched to reflect real-world deployment conditions.

#### Experimental Baseline (Mohammed et al., 2020)

A replication of Mohammed et al.'s statistical moment approach (Mean, Std Dev, Skewness, Kurtosis in time and frequency domains) was conducted as a controlled experiment:

| Phase | Configuration | Overall Accuracy | Abnormal Recall |
| :--- | :--- | :--- | :--- |
| Phase 1 (unweighted) | Standard loss | 81% | 21–26% — missing ~74–79% of faults |
| Phase 2 (balanced) | `class_weight='balanced'` | 77% | 49–61% — recall effectively doubled |

Feature importance analysis confirmed frequency standard deviation (`f_std`, ~0.26) and time standard deviation (`t_std`, ~0.24) as the strongest fault discriminators, followed by time kurtosis (`t_kurt`, ~0.15).

---

## Results Summary

| Model | Overall Accuracy | Macro F1 | Abnormal Recall (best) | Abnormal Recall (worst) |
| :--- | :--- | :--- | :--- | :--- |
| **ResNet18 CNN** | **99.75%** | **99.18%** | 100% (Machine 3) | 96.60% (Machine 2) |
| Random Forest (Track B) | ~95% | — | ~24% | ~16% |
| RF Baseline — weighted | 77% | — | 61% | 49% |

The CNN's Focal Loss + SpecAugment combination decisively overcame the class imbalance problem that imposed a hard ceiling on both Random Forest configurations. Machine 2 — the most challenging class due to its transient spike signature — saw fault recall jump from ~24% (RF) to **96.60%** (CNN).

---

## References

- GeeksforGeeks. (n.d.). *Window sliding technique.* https://www.geeksforgeeks.org/dsa/window-sliding-technique/
- Hafiz, N. F. M., et al. (2025). Machine learning framework for industrial machine sound classification in predictive maintenance. *IEEE Access, 13*, 154960–154975. https://doi.org/10.1109/ACCESS.2025.3601999
- Mohammed, T. S., et al. (2020). Fault diagnosis of rotating machine based on audio signal recognition system. *International Journal of Simulation: Systems, Science & Technology, 21*(2).
- Nichols, J. (2026). Preprocessing audio data for machine learning. *Medium.* https://medium.com/@nichojo89/preprocessing-audio-data-for-machine-learning-573f61b9d66d
- Shubita, R. R., Alsadeh, A. S., & Khater, I. M. (2023). Fault detection in rotating machinery based on sound signal using edge machine learning. *IEEE Access, 11*, 6665–6672. https://doi.org/10.1109/ACCESS.2023.3237074
