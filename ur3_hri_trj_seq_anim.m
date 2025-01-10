clc
clear all
close all

%% Animation
animation_on = 1;

% 0: robotlike, 1: humanlike
humanlike = 1;

if humanlike == 0
    % 0: non, 1: gripper
    tool_model = 1;
end

% 0: no, 1: yes
human_participant = 1;

%% Via-Points in time

% 1: horizontal, 2: vertical
robot_sequence = [1 0 1 0];
% robot_sequence = [0 1 0 1];
% robot_sequence = [1 1];

human_sequence = [1 0 0 1];

speed_ratio = [1 0.5 1.5];
% speed_ratio = [1 0.5];
% speed_ratio = [1];

trn_time = 3; % transition time
wait_time = 0.1; % waiting time

ft_time = 4; % one full-swing time
ft_dist = deg2rad(35); % one full-swing distance

cycle_num = 3;

%% Trajectory interpolation

% 0: minimum-jerk, 1: trapezoidal
trj_type = 0;

if trj_type == 1
    % tf = 2*ta + ts, ratio = ts/ta
    trapez_ratio = 8;
else
    % 0: unconstrained mj, 1: constrained mj, 2: separated mj
    mj_constrained = 2;
end

tstp = 0.1;

%%

deg1 = 23.5928; % asind(0.08535/0.21325);
% deg2 = 14.2348; % asind(0.11235/(0.24365+0.21325))

sequence_num = length(robot_sequence);
speed_num = length(speed_ratio);

point_num = sequence_num*(speed_num*(2*cycle_num+1)+2)+2;

tp_time = ft_time;

T = 0;
Pos = ones(6,point_num).*deg2rad([-90;-180;deg1;180-deg1;-90;0]);
PosH = zeros(2,point_num);

if trj_type == 0

    if mj_constrained == 1
        Vel = zeros(6,point_num);
        Acc = zeros(6,point_num);
    else
        Vel = zeros(6,2);
        Acc = zeros(6,2);
    end

    if mj_constrained == 2
        Ctrn = zeros(6,6,sequence_num+1);
        Cswg = zeros(6*2*cycle_num,6,sequence_num*speed_num);
        
        Ttrn = [0 trn_time];
        Tswg = zeros(1,2*cycle_num+1,sequence_num*speed_num);
        Tstep = zeros(1,sequence_num*(2*speed_num+2)+2);
        TOstep = zeros(1,sequence_num*4+2);
        Vstep = zeros(6,sequence_num*(2*speed_num+2)+2);
        Astep = zeros(6,sequence_num*(2*speed_num+2)+2);
    end
else
    Vel = zeros(6,point_num);
    Acc = zeros(6,point_num);
end
VelH = zeros(6,point_num);
AccH = zeros(6,point_num);

addpath('./funcs');

% initial
count = 2;
T(count) = trn_time;
Pos(:,count) = Pos(:,1);
if robot_sequence(1) == 1
    Pos(2,count) = Pos(2,count) - ft_dist;
else
    Pos(1,count) = Pos(1,count) - ft_dist;
end
if human_sequence(1) == 1
    PosH(2,count) = PosH(2,count) - ft_dist;
else
    PosH(1,count) = PosH(1,count) - ft_dist;
end
if ((trj_type == 0) && (mj_constrained == 2))
    step_count = 2;
    Tstep(step_count) = T(count);
    ostep_count = 2;
    TOstep(ostep_count) = T(count);
    Ptrn = Pos(:,1:2);
    for j = 1:6
        Ctrn(:,j,1) = minimumJerkCoefficient1DOF(Ttrn,Ptrn(j,:),Vel(j,:),Acc(j,:));
    end
end
for k = 1:sequence_num
    % wait
    count = count + 1;
    T(count) = T(count - 1) + wait_time;
    Pos(:,count) = Pos(:,count - 1);
    PosH(:,count) = PosH(:,count - 1);
    if ((trj_type == 0) && (mj_constrained == 2))
        step_count = step_count + 1;
        Tstep(step_count) = T(count);
        ostep_count = ostep_count + 1;
        TOstep(ostep_count) = T(count);
    end

    for w = 1:speed_num

        for i = 1:2*cycle_num
            % half-motion
            tp_time = ft_time*speed_ratio(w);

            count = count + 1;
            T(count) = T(count - 1) + tp_time;
            Pos(:,count) = Pos(:,1);
            if rem(i,2) == 1
                if robot_sequence(k) == 1
                    Pos(2,count) = Pos(2,1) + ft_dist;
                else
                    Pos(1,count) = Pos(1,1) + ft_dist;
                end
                if human_sequence(k) == 1
                    PosH(2,count) = PosH(2,1) + ft_dist;
                else
                    PosH(1,count) = PosH(1,1) + ft_dist;
                end
            end
            if rem(i,2) == 0
                if robot_sequence(k) == 1
                    Pos(2,count) = Pos(2,1) - ft_dist;
                else
                    Pos(1,count) = Pos(1,1) - ft_dist;
                end
                if human_sequence(k) == 1
                    PosH(2,count) = PosH(2,1) - ft_dist;
                else
                    PosH(1,count) = PosH(1,1) - ft_dist;
                end
            end
        end
        if ((trj_type == 0) && (mj_constrained == 2))
            step_count = step_count + 1;
            Tstep(step_count) = T(count);
            if w == speed_num
                ostep_count = ostep_count + 1;
                TOstep(ostep_count) = T(count);
            end
            Tswg(:,:,(k-1)*speed_num+w) = T(count-2*cycle_num:count)-T(count-2*cycle_num);
            Pswg = Pos(:,count-2*cycle_num:count);
            for j = 1:6
                Cswg(:,j,(k-1)*speed_num+w) = minimumJerkCoefficient1DOF(Tswg(:,:,(k-1)*speed_num+w),Pswg(j,:),Vel(j,:),Acc(j,:));
            end
        end

        % wait
        count = count + 1;
        T(count) = T(count - 1) + wait_time;
        Pos(:,count) = Pos(:,count - 1);
        PosH(:,count) = PosH(:,count - 1);
        if ((trj_type == 0) && (mj_constrained == 2))
            step_count = step_count + 1;
            Tstep(step_count) = T(count);
            if w == speed_num
                ostep_count = ostep_count + 1;
                TOstep(ostep_count) = T(count);
            end
        end

    end

    % transition
    count = count + 1;
    T(count) = T(count - 1) + trn_time;
    Pos(:,count) = Pos(:,1);
    PosH(:,count) = PosH(:,1);
    if k ~= sequence_num
        if robot_sequence(k + 1) == 1
            Pos(2,count) = Pos(2,1) - ft_dist;
        else
            Pos(1,count) = Pos(1,1) - ft_dist;
        end
        if human_sequence(k + 1) == 1
            PosH(2,count) = PosH(2,1) - ft_dist;
        else
            PosH(1,count) = PosH(1,1) - ft_dist;
        end
    end
    if ((trj_type == 0) && (mj_constrained == 2))
        step_count = step_count + 1;
        Tstep(step_count) = T(count);
        ostep_count = ostep_count + 1;
        TOstep(ostep_count) = T(count);
        Ptrn = Pos(:,count-1:count);
        for j = 1:6
            Ctrn(:,j,k+1) = minimumJerkCoefficient1DOF(Ttrn,Ptrn(j,:),Vel(j,:),Acc(j,:));
        end
    end

end

% figure
% tiledlayout(2,1)
% ax1 = nexttile;
% plot(T,rad2deg(Pos(2,:)),'o-')
% title(ax1,"Horizontal motion (joint 2)")
% ax2 = nexttile;
% plot(T,rad2deg(Pos(1,:)),'o-')
% title(ax2,"Vertical motion (joint 1)")
% 
% figure
% tiledlayout(2,2)
% ax1 = nexttile;
% plot(T,rad2deg(Pos(2,:)),'o-')
% title(ax1,"Horizontal motion (joint 2)")
% ax2 = nexttile;
% plot(T,rad2deg(Pos(1,:)),'o-')
% title(ax2,"Vertical motion (joint 1)")
% ax3 = nexttile;
% if trj_type == 0 && mj_constrained == 0
%     plot([T(1) T(end)],rad2deg(Vel(2,:)),'o')
% elseif trj_type == 0 && mj_constrained == 2
%     plot(Tstep,rad2deg(Vstep(2,:)),'o')
% else
%     plot(T,rad2deg(Vel(2,:)),'o')
% end
% title(ax3,"Horizontal velocity (joint 2)")
% ax4 = nexttile;
% if trj_type == 0 && mj_constrained == 0
%     plot([T(1) T(end)],rad2deg(Vel(1,:)),'o')
% elseif trj_type == 0 && mj_constrained == 2
%     plot(Tstep,rad2deg(Vstep(1,:)),'o')
% else
%     plot(T,rad2deg(Vel(1,:)),'o')
% end
% title(ax4,"Vertical velocity (joint 1)")

% n = point_num-1;  % n= number of trajectories 

if trj_type == 0
    if mj_constrained ~= 2
        Cj = zeros(6*(point_num-1),6);
        for j = 1:6
            if mj_constrained == 1
                Cj(:,j) = minimumJerkConstrCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
            else
                Cj(:,j) = minimumJerkCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
            end
        end
    end
end

total_step = length(T(1):tstp:T(point_num));

t = T(1);

if ((trj_type == 0) && (mj_constrained == 2))
    t_step = t;
    stepcount_out = 1;
    stepcount_in = 1;
    trncount = 1;
    swgcount = 0;
    cyccount = 0;
end

q_pos = zeros(6,total_step);
q_vel = zeros(6,total_step);
q_acc = zeros(6,total_step);
q_jer = zeros(6,total_step);

swing_state = 0;

if animation_on == 1
    CHj(:,1) = minimumJerkConstrCoefficient1DOF(T,PosH(1,:),VelH(1,:),AccH(1,:));
    CHj(:,2) = minimumJerkConstrCoefficient1DOF(T,PosH(2,:),VelH(2,:),AccH(2,:));

    [a_i,alpha_i,d_i,theta_i_O] = kinematicParametersUR3;
    
    TI_Base = [eye(3),[0;0;-0.80];zeros(1,3),1];
    TI_B = [rot_x(-pi/2)*rot_z(pi/2),[0;+0.08;0];zeros(1,3),1];
    TB_0 = [rot_z(pi),zeros(3,1);zeros(1,3),1];
    TI_0 = TI_B*TB_0;
    TI_i = zeros(4,4,6);
    if humanlike == 0
        Tn_sC   = [eye(3),[0;0;0.0085];zeros(1,3),1];
        TsC_S   = [eye(3),[0;0;0.0375];zeros(1,3),1];
        TS_gC   = [eye(3),[0;0;0.0111];zeros(1,3),1];
        TgC_gB  = [eye(3),[0;0;0.0900];zeros(1,3),1];
        TgB_E   = [eye(3),[0;0;0.0403];zeros(1,3),1];
        TgB_oKa = [rot_y(pi/4),[0.035;0;-0.035];zeros(1,3),1];
        TgB_oKb = [rot_z(pi)*rot_y(pi/4),[-0.035;0;-0.035];zeros(1,3),1];
        TgB_oFa = [rot_y(pi/2),[0.06;0;-0.015];zeros(1,3),1];
        TgB_oFb = [rot_z(pi)*rot_y(pi/2),[-0.06;0;-0.015];zeros(1,3),1];
        TgB_iKa = [rot_y(pi/2.25),[0.02;0;-0.025];zeros(1,3),1];
        TgB_iKb = [rot_z(pi)*rot_y(pi/2.25),[-0.02;0;-0.025];zeros(1,3),1];
        TgB_iFa = [rot_y(pi/2),[0.065;0;0.035];zeros(1,3),1];
        TgB_iFb = [rot_z(pi)*rot_y(pi/2),[-0.065;0;0.035];zeros(1,3),1];
    else
        TH = [eye(3),[0;0;0.16];zeros(1,3),1];
        TgH = [eye(3),[0;0;0.2244];zeros(1,3),1];
    end
    if human_participant == 1
        ThA = [rot_z(pi/2)*rot_x(-pi/2),[2;0.08+d_i(1);0.0];zeros(1,3),1];
    end

    if humanlike == 1
        dx_e = 0.000;
        dy_e = -0.037;
        dz_e = 0.200;
    else
        if tool_model == 1
            dx_e = 0.000;
            dy_e = 0.000;
            dz_e = Tn_sC(3,4)+TsC_S(3,4)+TS_gC(3,4)+TgC_gB(3,4)+TgB_E(3,4);
        else
            dx_e = 0.000;
            dy_e = 0.000;
            dz_e = 0.000;
        end
    end

    Tn_E = [eye(3),[dx_e;dy_e;dz_e];zeros(1,3),1];

    h_pos = 0;
    v_pos = 0;

    t_pos = zeros(3,total_step);
    r_pos = zeros(3,total_step);
    
    euler_set = 1; % 1: ZYZ, 2: ZYX, 3: XYZ, 4:ZXZ

    tool_alpha = 1;

    % solidness: (0,1)
    model_alpha = 1;

    addpath('./mat');
    
    sB = load('alu_profile.mat');
    s0 = load('base.mat');
    s1 = load('shoulder.mat');
    s2 = load('upperarm.mat');
    s3 = load('forearm.mat');
    s4 = load('wrist1.mat');
    s5 = load('wrist2.mat');
    s6 = load('wrist3.mat');
    if humanlike == 0
        sSC = load('fts150_coupling.mat');
        sS = load('fts150.mat');
        sGC = load('gripper_coupling.mat');
        sGB = load('arg2f_85_base_link_simp.mat');
        sOK = load('arg2f_85_outer_knuckle_simp.mat');
        sOF = load('arg2f_85_outer_finger_simp.mat');
        sIK = load('arg2f_85_inner_knuckle_simp.mat');
        sIF = load('arg2f_85_inner_finger_pad_simp.mat');
    else
        sH = load('head.mat');
        sGH = load('glove.mat');
    end
    sCF = load('coord_frame.mat');
    if human_participant == 1
        sHA = load('human_arm.mat');
    end
    
    % Plot imported CAD data from file
    animFig = figure('Name','Robot Plot');
    animPlot = axes('Parent',animFig);
    
    pB = patch(animPlot,'Faces', sB.modelstruct.F, 'Vertices' ,sB.modelstruct.V);
    set(pB, 'facec', 'flat');               % Set the face color flat
    set(pB, 'FaceColor', sB.modelstruct.C);	% Set the face color
    set(pB, 'EdgeColor', 'none');        	% Set the edge color
    
    p0 = patch(animPlot,'Faces', s0.modelstruct.F, 'Vertices' ,s0.modelstruct.V);
    set(p0, 'facec', 'flat');               % Set the face color flat
    set(p0, 'FaceColor', s0.modelstruct.C);	% Set the face color
    set(p0, 'EdgeColor', 'none');        	% Set the edge color
    
    p1 = patch(animPlot,'Faces', s1.modelstruct.F, 'Vertices' ,s1.modelstruct.V);
    set(p1, 'facec', 'flat');               % Set the face color flat
    set(p1, 'FaceVertexCData', s1.modelstruct.C);	% Set the face color
    set(p1, 'EdgeColor', 'none');        	% Set the edge color
    
    p2 = patch(animPlot,'Faces', s2.modelstruct.F, 'Vertices' ,s2.modelstruct.V);
    set(p2, 'facec', 'flat');               % Set the face color flat
    set(p2, 'FaceVertexCData', s2.modelstruct.C);	% Set the face color
    set(p2, 'EdgeColor', 'none');        	% Set the edge color
    
    p3 = patch(animPlot,'Faces', s3.modelstruct.F, 'Vertices' ,s3.modelstruct.V);
    set(p3, 'facec', 'flat');               % Set the face color flat
    set(p3, 'FaceVertexCData', s3.modelstruct.C);	% Set the face color
    set(p3, 'EdgeColor', 'none');        	% Set the edge color
    
    p4 = patch(animPlot,'Faces', s4.modelstruct.F, 'Vertices' ,s4.modelstruct.V);
    set(p4, 'facec', 'flat');               % Set the face color flat
    set(p4, 'FaceVertexCData', s4.modelstruct.C);	% Set the face color
    set(p4, 'EdgeColor', 'none');        	% Set the edge color
    
    p5 = patch(animPlot,'Faces', s5.modelstruct.F, 'Vertices' ,s5.modelstruct.V);
    set(p5, 'facec', 'flat');               % Set the face color flat
    set(p5, 'FaceVertexCData', s5.modelstruct.C);	% Set the face color
    set(p5, 'EdgeColor', 'none');        	% Set the edge color
    
    p6 = patch(animPlot,'Faces', s6.modelstruct.F, 'Vertices' ,s6.modelstruct.V);
    set(p6, 'facec', 'flat');               % Set the face color flat
    set(p6, 'FaceVertexCData', s6.modelstruct.C);	% Set the face color
    set(p6, 'EdgeColor', 'none');        	% Set the edge color
    
    if humanlike == 0
        pSC = patch(animPlot,'Faces', sSC.modelstruct.F, 'Vertices' ,sSC.modelstruct.V);
        set(pSC, 'facec', 'flat');               % Set the face color flat
        set(pSC, 'FaceVertexCData', sSC.modelstruct.C);	% Set the face color
        set(pSC, 'EdgeColor', 'none');        	% Set the edge color
        
        pS = patch(animPlot,'Faces', sS.modelstruct.F, 'Vertices' ,sS.modelstruct.V);
        set(pS, 'facec', 'flat');               % Set the face color flat
        set(pS, 'FaceVertexCData', sS.modelstruct.C);	% Set the face color
        set(pS, 'EdgeColor', 'none');        	% Set the edge color
        
        pGC = patch(animPlot,'Faces', sGC.modelstruct.F, 'Vertices' ,sGC.modelstruct.V);
        set(pGC, 'facec', 'flat');               % Set the face color flat
        set(pGC, 'FaceColor', sGC.modelstruct.C);	% Set the face color
        set(pGC, 'EdgeColor', 'none');        	% Set the edge color
        
        pGB = patch(animPlot,'Faces', sGB.modelstruct.F, 'Vertices' ,sGB.modelstruct.V);
        set(pGB, 'facec', 'flat');               % Set the face color flat
        set(pGB, 'FaceVertexCData', sGB.modelstruct.C);	% Set the face color
        set(pGB, 'EdgeColor', 'none');        	% Set the edge color
        
        pOKa = patch(animPlot,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
        set(pOKa, 'facec', 'flat');               % Set the face color flat
        set(pOKa, 'FaceColor', sOK.modelstruct.C);	% Set the face color
        set(pOKa, 'EdgeColor', 'none');        	% Set the edge color
        
        pOKb = patch(animPlot,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
        set(pOKb, 'facec', 'flat');               % Set the face color flat
        set(pOKb, 'FaceColor', sOK.modelstruct.C);	% Set the face color
        set(pOKb, 'EdgeColor', 'none');        	% Set the edge color
        
        pOFa = patch(animPlot,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
        set(pOFa, 'facec', 'flat');               % Set the face color flat
        set(pOFa, 'FaceColor', sOF.modelstruct.C);	% Set the face color
        set(pOFa, 'EdgeColor', 'none');        	% Set the edge color
        
        pOFb = patch(animPlot,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
        set(pOFb, 'facec', 'flat');               % Set the face color flat
        set(pOFb, 'FaceColor', sOF.modelstruct.C);	% Set the face color
        set(pOFb, 'EdgeColor', 'none');        	% Set the edge color
        
        pIKa = patch(animPlot,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
        set(pIKa, 'facec', 'flat');               % Set the face color flat
        set(pIKa, 'FaceColor', sIK.modelstruct.C);	% Set the face color
        set(pIKa, 'EdgeColor', 'none');        	% Set the edge color
        
        pIKb = patch(animPlot,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
        set(pIKb, 'facec', 'flat');               % Set the face color flat
        set(pIKb, 'FaceColor', sIK.modelstruct.C);	% Set the face color
        set(pIKb, 'EdgeColor', 'none');        	% Set the edge color
        
        pIFa = patch(animPlot,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
        set(pIFa, 'facec', 'flat');               % Set the face color flat
        set(pIFa, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
        set(pIFa, 'EdgeColor', 'none');        	% Set the edge color
        
        pIFb = patch(animPlot,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
        set(pIFb, 'facec', 'flat');               % Set the face color flat
        set(pIFb, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
        set(pIFb, 'EdgeColor', 'none');        	% Set the edge color
    else
        pGH = patch(animPlot,'Faces', sGH.modelstruct.F, 'Vertices' ,sGH.modelstruct.V);
        set(pGH, 'facec', 'flat');               % Set the face color flat
        set(pGH, 'FaceColor', sGH.modelstruct.C);	% Set the face color
        set(pGH, 'EdgeColor', 'none');        	% Set the edge color
        
        pH = patch(animPlot,'Faces', sH.modelstruct.F, 'Vertices' ,sH.modelstruct.V);
        set(pH, 'facec', 'flat');               % Set the face color flat
        set(pH, 'FaceColor', sH.modelstruct.C);	% Set the face color
        set(pH, 'EdgeColor', 'none');        	% Set the edge color
    end
    
    pCF = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
    set(pCF, 'facec', 'flat');                      % Set the face color flat
    set(pCF, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
    set(pCF, 'EdgeColor', 'none');
    
    if human_participant == 1
        pHA = patch(animPlot,'Faces', sHA.modelstruct.F, 'Vertices' ,sHA.modelstruct.V);
        set(pHA, 'facec', 'flat');               % Set the face color flat
        set(pHA, 'FaceColor', sHA.modelstruct.C);	% Set the face color
        set(pHA, 'EdgeColor', 'none');        	% Set the edge color
    end
    
    light(animPlot)             % add a default light
    view(animPlot,[-225,30])	% Isometric view
    grid(animPlot,'on')
    xlabel(animPlot,'X'),ylabel(animPlot,'Y'),zlabel(animPlot,'Z')
    set(animPlot,	'Projection','perspective', ...
                    'PlotBoxAspectRatio',[1 1 1], ...
                    'DataAspectRatio',[1 1 1]);
    
    temp = [sB.modelstruct.V,ones(size(sB.modelstruct.V(:,1)))]*TI_Base';
    set(pB,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    
    temp = [s0.modelstruct.V,ones(size(s0.modelstruct.V(:,1)))]*TI_0';
    set(p0,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    
    if humanlike == 1
        temp = [sH.modelstruct.V,ones(size(sH.modelstruct.V(:,1)))]*TH';
        set(pH,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    end
end

for k = 1:total_step

    if trj_type == 0 && mj_constrained == 2

        if t > TOstep(stepcount_out + 1)
            stepcount_out = stepcount_out + 1;
            swing_state = 0;
            if rem(stepcount_out,4) == 1
                trncount = trncount + 1;
                cyccount = 0;
            elseif rem(stepcount_out,4) == 3
                swing_state = 1;
            end
            t_step = 0;
        end

        if t > Tstep(stepcount_in + 1)
            stepcount_in = stepcount_in + 1;
            if swing_state == 1
                cyccount = cyccount + 1;
                if rem(cyccount,2) == 1
                    swgcount = swgcount + 1;
                    t_step = 0;
                end
            end
        end

        for j = 1:6
            if rem(stepcount_out,4) == 1
                % transition
                [q_pos(j,k),q_vel(j,k),q_acc(j,k),q_jer(j,k)] = minimumJerkPolynomial1DOF(t_step,Ttrn,Ctrn(:,j,trncount));
            elseif rem(stepcount_out,4) == 3
                if rem(cyccount,2) == 1
                    % swing
                    [q_pos(j,k),q_vel(j,k),q_acc(j,k),q_jer(j,k)] = minimumJerkPolynomial1DOF(t_step,Tswg(:,:,swgcount),Cswg(:,j,swgcount));
                else
                    % wait
                    q_pos(j,k) = q_pos(j,k-1);
                    q_vel(j,k) = 0;
                    q_acc(j,k) = 0;
                    q_jer(j,k) = 0;
                end
            else
                % wait
                q_pos(j,k) = q_pos(j,k-1);
                q_vel(j,k) = 0;
                q_acc(j,k) = 0;
                q_jer(j,k) = 0;
            end
        end
        
        t_step = round(t_step + tstp,3);

    else

        for j = 1:6
            if trj_type == 1
                [q_pos(j,k),q_vel(j,k),q_acc(j,k),q_jer(j,k)] = p2pTrjTrap1DOF(t,T,Pos(j,:),trapez_ratio);
            else
                [q_pos(j,k),q_vel(j,k),q_acc(j,k),q_jer(j,k)] = minimumJerkPolynomial1DOF(t,T,Cj(:,j));
            end
        end

    end

    if animation_on == 1
        if human_participant == 1
            h_pos = minimumJerkPolynomial1DOF(t,T,CHj(:,2));
            v_pos = -minimumJerkPolynomial1DOF(t,T,CHj(:,1));
        end
    
        [T0_h,~] = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+q_pos(:,k));
    
        for i = 1:6
            TI_i(:,:,i) = TI_0*T0_h(:,:,i+1);
        end
    
        if humanlike == 0
            TI_sC = TI_i(:,:,6)*Tn_sC;
            TI_S = TI_sC*TsC_S;
            TI_gC = TI_S*TS_gC;
            TI_gB = TI_gC*TgC_gB;
            TI_oKa = TI_gB*TgB_oKa;
            TI_oKb = TI_gB*TgB_oKb;
            TI_oFa = TI_gB*TgB_oFa;
            TI_oFb = TI_gB*TgB_oFb;
            TI_iKa = TI_gB*TgB_iKa;
            TI_iKb = TI_gB*TgB_iKb;
            TI_iFa = TI_gB*TgB_iFa;
            TI_iFb = TI_gB*TgB_iFb;
        else
            TI_gH = TI_i(:,:,6)*TgH;
        end
        if human_participant == 1
            TI_hA = ThA*[rot_y(h_pos)*rot_x(v_pos),zeros(3,1);zeros(1,3),1];
        end
    
        TI_E = TI_i(:,:,6)*Tn_E;
    
        t_pos(:,k) = TI_E(1:3,4);
        r_pos(:,k) = getEulerPosVec(TI_E(1:3,1:3),euler_set);
    
        % transformation
        temp = [s1.modelstruct.V,ones(size(s1.modelstruct.V(:,1)))]*TI_i(:,:,1)';
        set(p1,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        temp = [s2.modelstruct.V,ones(size(s2.modelstruct.V(:,1)))]*TI_i(:,:,2)';
        set(p2,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        temp = [s3.modelstruct.V,ones(size(s3.modelstruct.V(:,1)))]*TI_i(:,:,3)';
        set(p3,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        temp = [s4.modelstruct.V,ones(size(s4.modelstruct.V(:,1)))]*TI_i(:,:,4)';
        set(p4,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        temp = [s5.modelstruct.V,ones(size(s5.modelstruct.V(:,1)))]*TI_i(:,:,5)';
        set(p5,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        temp = [s6.modelstruct.V,ones(size(s6.modelstruct.V(:,1)))]*TI_i(:,:,6)';
        set(p6,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        if humanlike == 0
            temp = [sSC.modelstruct.V,ones(size(sSC.modelstruct.V(:,1)))]*TI_sC';
            set(pSC,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sS.modelstruct.V,ones(size(sS.modelstruct.V(:,1)))]*TI_S';
            set(pS,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sGC.modelstruct.V,ones(size(sGC.modelstruct.V(:,1)))]*TI_gC';
            set(pGC,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sGB.modelstruct.V,ones(size(sGB.modelstruct.V(:,1)))]*TI_gB';
            set(pGB,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKa';
            set(pOKa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKb';
            set(pOKb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFa';
            set(pOFa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFb';
            set(pOFb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKa';
            set(pIKa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKb';
            set(pIKb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFa';
            set(pIFa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
            temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFb';
            set(pIFb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
        else
            temp = [sGH.modelstruct.V,ones(size(sGH.modelstruct.V(:,1)))]*TI_gH';
            set(pGH,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
        end
        temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_E';
        set(pCF,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        if human_participant == 1
            temp = [sHA.modelstruct.V,ones(size(sHA.modelstruct.V(:,1)))]*TI_hA';
            set(pHA,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
        end

        hold(animPlot,'on')
        grid(animPlot,'on')
        plot3(animPlot,t_pos(1,1:k),t_pos(2,1:k),t_pos(3,1:k),'b')
        hold(animPlot,'off')
        drawnow
        
        if model_alpha < 0.5
            hold(animPlot,'on')
            plot3(animPlot,TI_E(1,4),TI_E(2,4),TI_E(3,4),'b*')
            plot3(animPlot,TI_i(1,4,6),TI_i(2,4,6),TI_i(3,4,6),'r*')
            plot3(animPlot,TI_i(1,4,5),TI_i(2,4,5),TI_i(3,4,5),'g*')
            plot3(animPlot,TI_i(1,4,4),TI_i(2,4,4),TI_i(3,4,4),'m*')
            plot3(animPlot,TI_i(1,4,3),TI_i(2,4,3),TI_i(3,4,3),'c*')
            plot3(animPlot,TI_i(1,4,2),TI_i(2,4,2),TI_i(3,4,2),'k*')
            hold(animPlot,'off')
        end
    end

    t = round(t + tstp,3);

end

figure
subplot(2,1,1)
plot(T(1):tstp:T(point_num),rad2deg(q_pos(2,:)))
hold on
plot(T,rad2deg(Pos(2,:)),'ro')
title("Horizontal motion (joint 2)")
subplot(2,1,2)
plot(T(1):tstp:T(point_num),rad2deg(q_pos(1,:)))
hold on
plot(T,rad2deg(Pos(1,:)),'ro')
title("Vertical motion (joint 1)")

jnt1Fig = figure('Name','Joint 2 Data Plot');
jnt1Fig.Position = [550 120 850 450];

j1posax = subplot(4,2,1);
plot(j1posax,T(1):tstp:T(point_num),rad2deg(q_pos(2,:)))
hold on
plot(T,rad2deg(Pos(2,:)),'o')
hold off
grid(j1posax,'on')
xlabel(j1posax,'Time (sec)')
ylabel(j1posax,'Position (deg)');
j1velax = subplot(4,2,3);
plot(j1velax,T(1):tstp:T(point_num),rad2deg(q_vel(2,:)))
hold on
if trj_type == 0 && mj_constrained == 0
    plot([T(1) T(end)],rad2deg(Vel(2,:)),'o')
elseif trj_type == 0 && mj_constrained == 2
    plot(Tstep,rad2deg(Vstep(2,:)),'o')
else
    plot(T,rad2deg(Vel(2,:)),'o')
end
hold off
grid(j1velax,'on')
xlabel(j1velax,'Time (sec)')
ylabel(j1velax,'Velocity (deg/s)');
j1accax = subplot(4,2,5);
plot(j1accax,T(1):tstp:T(point_num),rad2deg(q_acc(2,:)))
hold on
if trj_type == 0 && mj_constrained == 0
    plot([T(1) T(end)],rad2deg(Acc(2,:)),'o')
elseif trj_type == 0 && mj_constrained == 2
    plot(Tstep,rad2deg(Astep(2,:)),'o')
else
    plot(T,rad2deg(Acc(2,:)),'o')
end
hold off
grid(j1accax,'on')
xlabel(j1accax,'Time (sec)')
ylabel(j1accax,'Acceleration (deg/s^2)');
j1jerax = subplot(4,2,7);
plot(j1jerax,T(1):tstp:T(point_num),rad2deg(q_jer(2,:)))
grid(j1jerax,'on')
xlabel(j1jerax,'Time (sec)')
ylabel(j1jerax,'Jerk (deg/s^3)');

j2posax = subplot(4,2,2);
plot(j2posax,T(1):tstp:T(point_num),rad2deg(q_pos(1,:)))
hold on
plot(T,rad2deg(Pos(1,:)),'o')
hold off
grid(j2posax,'on')
xlabel(j2posax,'Time (sec)')
ylabel(j2posax,'Position (deg)');
j2velax = subplot(4,2,4);
plot(j2velax,T(1):tstp:T(point_num),rad2deg(q_vel(1,:)))
hold on
if trj_type == 0 && mj_constrained == 0
    plot([T(1) T(end)],rad2deg(Vel(1,:)),'o')
elseif trj_type == 0 && mj_constrained == 2
    plot(Tstep,rad2deg(Vstep(1,:)),'o')
else
    plot(T,rad2deg(Vel(1,:)),'o')
end
hold off
grid(j2velax,'on')
xlabel(j2velax,'Time (sec)')
ylabel(j2velax,'Velocity (deg/s)');
j2accax = subplot(4,2,6);
plot(j2accax,T(1):tstp:T(point_num),rad2deg(q_acc(1,:)))
hold on
if trj_type == 0 && mj_constrained == 0
    plot([T(1) T(end)],rad2deg(Acc(1,:)),'o')
elseif trj_type == 0 && mj_constrained == 2
    plot(Tstep,rad2deg(Astep(1,:)),'o')
else
    plot(T,rad2deg(Acc(1,:)),'o')
end
hold off
grid(j2accax,'on')
xlabel(j2accax,'Time (sec)')
ylabel(j2accax,'Acceleration (deg/s^2)');
j2jerax = subplot(4,2,8);
plot(j2jerax,T(1):tstp:T(point_num),rad2deg(q_jer(1,:)))
grid(j2jerax,'on')
xlabel(j2jerax,'Time (sec)')
ylabel(j2jerax,'Jerk (deg/s^3)');

function output = rot_z(input)
    output = [  cos(input)	-sin(input)	0
                sin(input)	 cos(input)	0
                0            0          1];
end

function output = rot_y(input)
    output = [  cos(input)	0   sin(input)
                0           1   0
                -sin(input)	0	cos(input)];
end

function output = rot_x(input)
    output = [	1	0           0
                0	cos(input)  -sin(input)
                0	sin(input)	cos(input)];
end