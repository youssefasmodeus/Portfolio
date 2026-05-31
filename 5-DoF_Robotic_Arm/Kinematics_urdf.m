%% =========================================================
%  5-DOF RRRRR Arm – URDF-Merged IK Simulation
%
%  All parameters extracted from urdfnew.urdf:
%
%  Joint  | URDF origin (key value)        | Role in FK
%  -------|--------------------------------|-------------------
%  R1     | xyz="... ... 0.028"            | Base yaw, Zoffset
%  R2     | xyz="... 0.09355 ..."          | Shoulder, L_shldr
%  R3     | xyz="-0.014 0.14 0"            | Upper arm, L1
%  R4     | xyz="0.0928 ..."               | Forearm,   L2
%  R5     | xyz="0 0 0.0284"              | Wrist,     L3
%
%  Joint axes from URDF:
%    R1 axis="0 -1 0"  → Yaw   (mapped to Ryaw)
%    R2 axis="-1 0 0"  → Pitch (mapped to Rpitch, sign-flipped)
%    R3 axis="0 0 -1"  → Pitch (mapped to Rpitch, sign-flipped)
%    R4 axis="0 0 -1"  → Roll  (mapped to Rroll,  sign-flipped)
%    R5 axis="0 0 -1"  → Pitch (mapped to Rpitch, sign-flipped)
% =========================================================

clear; clc; close all;

%% ── 1. URDF-EXTRACTED PARAMETERS ───────────────────────
d2r = pi/180;
r2d = 180/pi;

% All lengths in cm (URDF is in metres, multiplied by 100)
Zoffset  =  2.8;   % R1 joint origin z = 0.028 m
L_shldr  =  9.4;   % R2 joint origin y = 0.09355 m  (shoulder-to-elbow offset)
L1       = 14.0;   % R3 joint origin x = 0.14 m     (upper arm)
L2       =  9.3;   % R4 joint origin x = 0.0928 m   (forearm)
L3       =  2.8;   % R5 joint origin z = 0.0284 m   (wrist)
Lg       =  5.0;   % Gripper (user-defined, not in URDF)

% Sign corrections for URDF joint axes (negative axis → flip q sign in FK)
% R2 axis="-1 0 0" → sign = -1
% R3 axis="0 0 -1" → sign = -1
% R4 axis="0 0 -1" → sign = -1
% R5 axis="0 0 -1" → sign = -1
AXIS_SIGN = [1, -1, -1, -1, -1];   % per joint

% Joint limits [deg] from URDF <limit lower upper>
jLimDeg = [-180  180 ;   % R1  lower=-3.1415 upper=3.1415
            -90   90 ;   % R2  lower=-1.5708 upper=1.5708
           -120  120 ;   % R3  lower=-2.0944 upper=2.0944
           -180  180 ;   % R4  lower=-3.1415 upper=3.1415
            -90   90 ];  % R5  lower=-1.5708 upper=1.5708

jNames = {'R1 (Base Yaw)','R2 (Shoulder)','R3 (Elbow)','R4 (Wrist Roll)','R5 (Wrist Pitch)'};

% Inertia data from URDF (mass in kg) — for reference / dynamics use
% L1_Ground: 0.144 kg | L2: 0.142 kg | L3: 0.191 kg
% L4: 0.057 kg        | L5: 0.029 kg | L6: 0.046 kg
mass = [0.144, 0.142, 0.191, 0.057, 0.029, 0.046];   % kg

fprintf('========================================\n');
fprintf('  5-DOF ARM – URDF-MERGED SIMULATION\n');
fprintf('========================================\n');
fprintf('Parameters from urdfnew.urdf:\n');
fprintf('  Zoffset  = %.1f cm  (R1 z-origin)\n', Zoffset);
fprintf('  L_shldr  = %.1f cm  (R2 y-origin, shoulder offset)\n', L_shldr);
fprintf('  L1       = %.1f cm  (R3 x-origin, upper arm)\n', L1);
fprintf('  L2       = %.1f cm  (R4 x-origin, forearm)\n', L2);
fprintf('  L3       = %.1f cm  (R5 z-origin, wrist)\n', L3);
fprintf('  Lg       = %.1f cm  (gripper, user-defined)\n', Lg);
fprintf('\nJoint limits (from URDF):\n');
for i = 1:5
    fprintf('  %s : [%+.0f°, %+.0f°]\n', jNames{i}, jLimDeg(i,1), jLimDeg(i,2));
end
fprintf('========================================\n\n');

%% ── 2. FORWARD KINEMATICS (URDF-accurate) ───────────────
% Uses rotation matrices matching each URDF joint axis direction.
% AXIS_SIGN flips the joint angle for joints with negative axes.
%
% Chain (positions in cm):
%   P0   = [0, 0, 0]           world origin
%   P1   = [0, 0, Zoffset]     base top (R1 pivot)
%   P2   = P1 + R1 * [0, L_shldr, 0]   shoulder to elbow along local Y
%   P3   = P2 + R1·R2 * [L1, 0, 0]     upper arm along local X
%   P4   = P3 + R1·R2·R3 * [L2, 0, 0]  forearm along local X
%   P5   = P4 + R1·R2·R3·R4 * [0, 0, L3]  wrist along local Z
%   Ptip = P5 + R_tip * [Lg, 0, 0]     gripper along local X

function [P_all, R_tip] = forward_kinematics(q, L_shldr, L1, L2, L3, Lg, Zoffset, AXIS_SIGN)
    % Apply URDF axis signs
    qs = q .* AXIS_SIGN(:);

    % Elementary rotation matrices
    Ryaw   = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
    Rpitch = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
    Rroll  = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];

    % Global frame orientations (accumulated)
    R1 = Ryaw(qs(1));                    % R1: base yaw
    R2 = R1 * Rpitch(qs(2));             % R2: shoulder pitch
    R3 = R2 * Rpitch(qs(3));             % R3: elbow pitch
    R4 = R3 * Rroll(qs(4));              % R4: wrist roll
    R5 = R4 * Rpitch(qs(5));             % R5: wrist pitch

    % Joint positions (cm)
    P0   = [0; 0; 0];
    P1   = [0; 0; Zoffset];              % base pivot
    P2   = P1 + R1  * [0; L_shldr; 0];  % shoulder offset along local Y
    P3   = P2 + R2  * [L1; 0; 0];       % upper arm along local X
    P4   = P3 + R3  * [L2; 0; 0];       % forearm along local X
    P5   = P4 + R4  * [0; 0; L3];       % wrist along local Z
    Ptip = P5 + R5  * [Lg; 0; 0];       % gripper along local X

    P_all = [P0'; P1'; P2'; P3'; P4'; P5'; Ptip'];
    R_tip = R5;
end

%% ── 3. INVERSE KINEMATICS (Geometric + URDF geometry) ──
% Treats the arm as: base yaw (R1) + planar 2R chain (R2,R3) + wrist (R4,R5)
% L_eff = L1 + L2 + L3 + Lg  (full reach in the pitch plane)

function [q, reachable] = inverse_kinematics(X, Y, Z, Roll_deg, Pitch_deg, ...
                                             L_shldr, L1, L2, L3, Lg, Zoffset, ...
                                             jLimDeg, AXIS_SIGN)
    d2r = pi/180;
    q   = zeros(5,1);

    % Effective reach arm (shoulder offset ignored for planar reach check)
    L_eff = L1 + L2 + L3 + Lg;

    % R1: base yaw — atan2(Y,X)
    q(1) = atan2(Y, X);

    % Planar distance from base axis and height
    R     = sqrt(X^2 + Y^2);
    Z_rel = Z - Zoffset;
    D2    = R^2 + Z_rel^2;
    D     = sqrt(D2);

    if D > L_eff || D < abs(L1 - (L2+L3+Lg))
        reachable = false;
        return;
    end
    reachable = true;

    % Effective forearm = L2+L3+Lg (lumped) for 2R cosine rule
    L_fore = L2 + L3 + Lg;

    % R3: elbow (cosine rule on triangle L1, L_fore, D)
    cos_q3 = (D2 - L1^2 - L_fore^2) / (2 * L1 * L_fore);
    cos_q3 = max(-1, min(1, cos_q3));
    q(3)   = -acos(cos_q3);   % elbow-down solution

    % R2: shoulder (elevation + geometric offset)
    alpha  = atan2(Z_rel, R);
    beta   = atan2(L_fore * sin(abs(q(3))), L1 + L_fore * cos(abs(q(3))));
    q(2)   = alpha + beta;

    % R4: wrist roll (user-specified)
    q(4)   = Roll_deg  * d2r;

    % R5: wrist pitch (user-specified, 0 = level)
    q(5)   = Pitch_deg * d2r;

    % Undo URDF axis sign before returning (q is in logical/MATLAB space)
    % The FK applies AXIS_SIGN internally, so IK returns raw q
    % Clamp to joint limits
    for i = 1:5
        q(i) = max(jLimDeg(i,1)*d2r, min(jLimDeg(i,2)*d2r, q(i)));
    end
end

%% ── 4. BUILD FIGURE ─────────────────────────────────────
fig = figure('Name','5-DOF URDF-Merged Arm – IK Simulation', ...
             'Color','#1a1a2e','NumberTitle','off','Position',[60 50 1750 920]);

ax = axes('Position',[0.03 0.07 0.63 0.88], ...
          'Color','#0d0d1a','XColor','w','YColor','w','ZColor','w', ...
          'GridColor',[0.3 0.3 0.4],'GridAlpha',0.5,'FontSize',11);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,45,25);
xlim(ax,[-40 40]); ylim(ax,[-40 40]); zlim(ax,[0 45]);
xlabel(ax,'X [cm]','Color','w','FontSize',13,'FontWeight','bold');
ylabel(ax,'Y [cm]','Color','w','FontSize',13,'FontWeight','bold');
zlabel(ax,'Z [cm]','Color','w','FontSize',13,'FontWeight','bold');
title(ax,'5-DOF URDF-Merged Arm – IK Simulation', ...
      'Color','#FF6B35','FontSize',15,'FontWeight','bold');

% Coordinate arrows
quiver3(ax,0,0,0,8,0,0,'r','LineWidth',3,'MaxHeadSize',0.5,'AutoScale','off');
quiver3(ax,0,0,0,0,8,0,'g','LineWidth',3,'MaxHeadSize',0.5,'AutoScale','off');
quiver3(ax,0,0,0,0,0,8,'b','LineWidth',3,'MaxHeadSize',0.5,'AutoScale','off');
text(ax,10,0,0,'X','Color','r','FontSize',13,'FontWeight','bold');
text(ax,0,10,0,'Y','Color','g','FontSize',13,'FontWeight','bold');
text(ax,0,0,10,'Z','Color','b','FontSize',13,'FontWeight','bold');

% Base disc (matches L1_Ground footprint)
td = linspace(0,2*pi,60);
fill3(ax,4*cos(td),4*sin(td),zeros(1,60),[0.2 0.2 0.35], ...
      'EdgeColor',[0.5 0.5 0.6],'FaceAlpha',0.9);

% Approximate workspace sphere
R_max = L_shldr + L1 + L2 + L3 + Lg;
[sx,sy,sz] = sphere(30);
surf(ax,R_max*sx,R_max*sy,R_max*sz+Zoffset, ...
     'FaceAlpha',0.04,'EdgeAlpha',0.06,'FaceColor',[0.4 0.7 1],'EdgeColor',[0.4 0.7 1]);

% Initial configuration
q_home = [0, 45, -90, 0, 0]' * d2r;
[P0, R0] = forward_kinematics(q_home, L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN);
t_sc = 3;   % triad scale [cm]
tip0 = P0(end,:)';

h_links  = plot3(ax,P0(:,1),P0(:,2),P0(:,3),'Color','#FF6B35','LineWidth',6);
h_joints = scatter3(ax,P0(2:end-1,1),P0(2:end-1,2),P0(2:end-1,3), ...
                    100,'w','filled','MarkerEdgeColor','#FF6B35');
h_tip    = scatter3(ax,P0(end,1),P0(end,2),P0(end,3), ...
                    180,'r','filled','MarkerEdgeColor','w','LineWidth',2);
h_target = plot3(ax,0,0,0,'w*','MarkerSize',18,'LineWidth',3,'Visible','off');
h_path   = plot3(ax,[],[],[],'Color',[0.2 0.85 1],'LineWidth',2,'LineStyle',':');

% Tip orientation triad
h_tx = plot3(ax,[tip0(1) tip0(1)+R0(1,1)*t_sc],[tip0(2) tip0(2)+R0(2,1)*t_sc], ...
                [tip0(3) tip0(3)+R0(3,1)*t_sc],'r','LineWidth',2.5);
h_ty = plot3(ax,[tip0(1) tip0(1)+R0(1,2)*t_sc],[tip0(2) tip0(2)+R0(2,2)*t_sc], ...
                [tip0(3) tip0(3)+R0(3,2)*t_sc],'g','LineWidth',2.5);
h_tz = plot3(ax,[tip0(1) tip0(1)+R0(1,3)*t_sc],[tip0(2) tip0(2)+R0(2,3)*t_sc], ...
                [tip0(3) tip0(3)+R0(3,3)*t_sc],'b','LineWidth',2.5);
h_tlbl = text(ax,tip0(1)+2,tip0(2)+2,tip0(3)+2,'', ...
              'Color','w','FontSize',9,'FontWeight','bold', ...
              'BackgroundColor',[0.7 0.2 0.1 0.9]);

% Joint label handles
for jj = 1:6
    h_jlbl(jj) = text(ax,0,0,0,'','Color','w','FontSize',8,'FontWeight','bold', ...
        'BackgroundColor',[0.12 0.12 0.20 0.88],'EdgeColor',[0.4 0.4 0.5]); %#ok<SAGROW>
end

%% ── 5. STATUS PANEL ─────────────────────────────────────
pn = uipanel('Position',[0.68 0.07 0.31 0.88], ...
    'BackgroundColor','#0d0d1a','ForegroundColor','#FF6B35', ...
    'BorderType','line','HighlightColor','#FF6B35', ...
    'Title',' STATUS ','FontSize',14,'FontWeight','bold');
ax_s = axes('Parent',pn,'Position',[0 0 1 1]);
axis(ax_s,'off'); xlim(ax_s,[0 1]); ylim(ax_s,[0 1]);

CH=[1 0.55 0.1]; CW=[0.88 0.88 0.88]; CV=[0.3 0.9 1.0];
CO=[0.2 0.9 0.3]; CE=[1 0.3 0.3]; fn='Courier New';

y=0.97; dy=0.050;
text(0.5,y,'5-DOF RRRRR ARM','Parent',ax_s,'FontSize',17,'FontWeight','bold', ...
    'HorizontalAlignment','center','Color',CH);
y=y-dy;
text(0.5,y,'URDF-merged | urdfnew.urdf','Parent',ax_s,'FontSize',10, ...
    'HorizontalAlignment','center','Color',[0.55 0.55 0.65]);
y=y-1.4*dy;

text(0.05,y,'END-EFFECTOR [cm]','Parent',ax_s,'FontSize',10,'FontWeight','bold','Color',CH);
y=y-dy;
txt_ee = text(0.07,y,'—','Parent',ax_s,'FontSize',10,'FontName',fn,'Color',CV,'FontWeight','bold');
y=y-1.3*dy;

text(0.05,y,'JOINT ANGLES','Parent',ax_s,'FontSize',10,'FontWeight','bold','Color',CH);
y=y-dy;
jC = {[1 0.4 0.4],[0.4 0.7 1],[0.4 1 0.6],[1 1 0.4],[0.85 0.4 1]};
for jj=1:5
    txt_j(jj) = text(0.07,y,'—','Parent',ax_s,'FontSize',9,'FontName',fn,'Color',jC{jj}); %#ok<SAGROW>
    y=y-dy;
end
y=y-0.3*dy;

text(0.05,y,'URDF LINK LENGTHS','Parent',ax_s,'FontSize',10,'FontWeight','bold','Color',CH);
y=y-dy;
text(0.07,y,sprintf('Zoff=%.1f  Lsh=%.1f  L1=%.1f  L2=%.1f  L3=%.1f  Lg=%.1f cm', ...
    Zoffset,L_shldr,L1,L2,L3,Lg), ...
    'Parent',ax_s,'FontSize',8,'FontName',fn,'Color',CW);
y=y-1.3*dy;

text(0.05,y,'IK ACCURACY','Parent',ax_s,'FontSize',10,'FontWeight','bold','Color',CH);
y=y-dy;
txt_err = text(0.07,y,'—','Parent',ax_s,'FontSize',11,'FontWeight','bold','FontName',fn,'Color',CO);
y=y-1.3*dy;

text(0.05,y,'STATUS','Parent',ax_s,'FontSize',10,'FontWeight','bold','Color',CH);
y=y-dy;
txt_st = text(0.07,y,'Ready','Parent',ax_s,'FontSize',10,'Color',CO);

%% ── 6. DRAW HELPER ──────────────────────────────────────
function update_visuals(q_now, L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN, t_sc, ...
                        h_links,h_joints,h_tip,h_tx,h_ty,h_tz,h_tlbl,h_jlbl, ...
                        txt_j,txt_ee,r2d)
    [P, R_t] = forward_kinematics(q_now,L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN);
    tip = P(end,:)';

    set(h_links,  'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
    set(h_joints, 'XData',P(2:end-1,1),'YData',P(2:end-1,2),'ZData',P(2:end-1,3));
    set(h_tip,    'XData',tip(1),'YData',tip(2),'ZData',tip(3));

    % Orientation triad
    set(h_tx,'XData',[tip(1) tip(1)+R_t(1,1)*t_sc],'YData',[tip(2) tip(2)+R_t(2,1)*t_sc],'ZData',[tip(3) tip(3)+R_t(3,1)*t_sc]);
    set(h_ty,'XData',[tip(1) tip(1)+R_t(1,2)*t_sc],'YData',[tip(2) tip(2)+R_t(2,2)*t_sc],'ZData',[tip(3) tip(3)+R_t(3,2)*t_sc]);
    set(h_tz,'XData',[tip(1) tip(1)+R_t(1,3)*t_sc],'YData',[tip(2) tip(2)+R_t(2,3)*t_sc],'ZData',[tip(3) tip(3)+R_t(3,3)*t_sc]);
    set(h_tlbl,'Position',tip+[2;1;2],'String',sprintf('EE [%.1f,%.1f,%.1f]',tip(1),tip(2),tip(3)));

    % 3D joint labels
    jLbls = {'R1','R2','R3','R4','R5','TIP'};
    for jj = 1:6
        set(h_jlbl(jj),'Position',[P(jj+1,1)+1 P(jj+1,2)+1 P(jj+1,3)+1.5], ...
            'String', jLbls{jj});
    end

    % Status panel joints
    jNms = {'R1(Yaw)','R2(Shldr)','R3(Elbow)','R4(Roll)','R5(Pitch)'};
    for jj=1:5
        set(txt_j(jj),'String',sprintf('%s = %+7.2f°', jNms{jj}, q_now(jj)*r2d));
    end
    set(txt_ee,'String',sprintf('(%.2f,  %.2f,  %.2f)', tip(1),tip(2),tip(3)));
end

%% ── 7. INITIAL DRAW ─────────────────────────────────────
q_current = q_home;
path_history = zeros(0,3);

update_visuals(q_current,L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN,t_sc, ...
               h_links,h_joints,h_tip,h_tx,h_ty,h_tz,h_tlbl,h_jlbl, ...
               txt_j,txt_ee,r2d);
drawnow;

%% ── 8. MAIN LOOP ────────────────────────────────────────
fprintf('\n========================================\n');
fprintf('  READY  –  commands below\n');
fprintf('========================================\n');

while ishandle(fig)
    fprintf('\n%s\n', repmat('-',1,52));
    fprintf('  1. IK  – enter target X,Y,Z\n');
    fprintf('  2. FK  – enter all 5 joint angles manually\n');
    fprintf('  3. HOME\n');
    fprintf('  4. Exit\n');
    choice = input('Select (1-4): ','s');

    if strcmp(choice,'4') || isempty(choice)
        fprintf('Exiting.\n'); break;

    elseif strcmp(choice,'1')
        tX = input('  X [cm]: ');
        tY = input('  Y [cm]: ');
        tZ = input('  Z [cm]: ');
        tR = input('  Wrist Roll  q4 [deg]: ');
        tP = input('  Wrist Pitch q5 [deg]: ');

        set(txt_st,'String','Solving IK…','Color',[1 0.8 0.2]); drawnow;

        [q_sol, ok] = inverse_kinematics(tX,tY,tZ,tR,tP, ...
                                         L_shldr,L1,L2,L3,Lg,Zoffset, ...
                                         jLimDeg,AXIS_SIGN);
        if ~ok
            fprintf('\n✗  UNREACHABLE\n');
            set(txt_st,'String','Target unreachable','Color',CE); continue;
        end

        [P_chk,~] = forward_kinematics(q_sol,L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN);
        fk_err = norm(P_chk(end,:) - [tX tY tZ]);
        fprintf('\n✓  IK Solved  (FK error = %.3f cm)\n', fk_err);
        if fk_err < 0.5
            set(txt_err,'String',sprintf('%.4f cm  ✓',fk_err),'Color',CO);
        else
            set(txt_err,'String',sprintf('%.4f cm  ~',fk_err),'Color',[1 0.8 0.2]);
        end
        set(h_target,'XData',tX,'YData',tY,'ZData',tZ,'Visible','on');
        mode_title = sprintf('IK → (%.1f, %.1f, %.1f) cm', tX,tY,tZ);

    elseif strcmp(choice,'2')
        fprintf('\n  Enter angles in DEGREES:\n');
        q_sol = zeros(5,1);
        lbs = {'q1 Base Yaw','q2 Shoulder','q3 Elbow','q4 WristRoll','q5 WristPitch'};
        for jj=1:5
            q_sol(jj) = input(sprintf('    %s [%+.0f..%+.0f]: ', ...
                              lbs{jj},jLimDeg(jj,1),jLimDeg(jj,2))) * d2r;
        end
        fk_err = 0;
        mode_title = 'Manual FK';

    elseif strcmp(choice,'3')
        q_sol = q_home;
        fk_err = 0;
        mode_title = 'HOME';
    else
        continue;
    end

    % Clamp
    for jj=1:5
        q_sol(jj) = max(jLimDeg(jj,1)*d2r, min(jLimDeg(jj,2)*d2r, q_sol(jj)));
    end

    % ── Smooth animation (sinusoidal easing) ────────────
    n_steps = 40;
    title(ax, mode_title,'Color','#FF6B35');
    set(txt_st,'String',sprintf('Moving → %s',mode_title),'Color',[1 0.8 0.2]);

    for step = 1:n_steps
        alpha_s = (1 - cos(pi*step/n_steps)) / 2;
        q_temp  = q_current + alpha_s*(q_sol - q_current);

        update_visuals(q_temp,L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN,t_sc, ...
                       h_links,h_joints,h_tip,h_tx,h_ty,h_tz,h_tlbl,h_jlbl, ...
                       txt_j,txt_ee,r2d);

        [P_t,~] = forward_kinematics(q_temp,L_shldr,L1,L2,L3,Lg,Zoffset,AXIS_SIGN);
        path_history(end+1,:) = P_t(end,:); %#ok<AGROW>
        set(h_path,'XData',path_history(:,1),'YData',path_history(:,2),'ZData',path_history(:,3));

        drawnow; pause(0.018);
        if ~ishandle(fig), return; end
    end

    q_current = q_sol;
    title(ax,'5-DOF URDF-Merged Arm – IK Simulation','Color','#FF6B35');
    set(txt_st,'String','Ready','Color',CO);
    fprintf('  ✓  Done\n');
end

fprintf('\nClosed.\n');