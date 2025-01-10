clc
clear all
close all

% write to csv
write_on = 0;

write_task = 1;

% 0: Base, 1: View (not implemented 2: Tool, 3: World)
feature = 1;

% 0: none, 1: sensor+gripper, 2: glove
ee_opt = 1;

% Coordinate frame for tool 0: TCP, 1: TCP_1, 2: TCP_2
TCP = 2;

% solidness: (0,1)
model_alpha = 1;

euler_set = 1; % 1: ZYZ, 2: ZYX, 3: XYZ, 4:ZXZ

% 0: minimum-jerk, 1: trapezoidal
trj_type = 0;

if trj_type == 1
    % tf = 2*ta + ts, ratio = ts/ta
    trapez_ratio = 8;
else
    % 0: unconstrained mj, 1: constrained mj
    mj_constrained = 1;
end

if write_on == 1
    tstp = 0.008;
else
    tstp = 0.1;
end

%%

m_approach = 1;
m_grab = 2;
m_transport_high = 3;
m_put = 4;
m_release = 5;
m_recede = 6;
m_pull = 7;
m_transport_low = 8;

% mot_seq = [m_approach,m_grab,m_transport_high,m_put,m_release,m_recede,m_approach,m_grab];
mot_seq = [m_approach,m_grab,m_transport_high,m_put,m_release,m_recede,m_approach,m_grab,m_pull,m_transport_low,m_release,m_recede];
% mot_seq = m_transport_high;

sequence_num = length(mot_seq);

% initial
Pos_ini = [-0.2000
            0.2800
            0.5600];

Orn_ini = deg2rad([ 90
                    90
                    180]);

% incremental values
% app_rec_dist = 0.05;    % m
app_rec_dist = 0.12;    % m
app_rec_duration = 2;   % sec
grab_rel_duration = 2;  % sec
hover_dist = 0.03;      % m
put_duration = 2;       % sec

Pos_ini = Pos_ini + [0;app_rec_dist;0];

transport_duration = [2 2];

Pos_transport = [   0.3000  0.0000      % x-axis
                    0.0000  0.0000      % y-axis
                    0.0000 -0.3000  ];  % z-axis

OrnMJ = [Orn_ini,Orn_ini,Orn_ini];

Orn = [Orn_ini,Orn_ini];

qPrev = deg2rad([-15;-95;-65;-15;15;270]); % reference configuration

Obj_Pos = Pos_ini + [0;0.12;0];

%% Mesh Initialization 
addpath('./funcs');
addpath('./mat');

if feature == 1
    sB = load('alu_profile.mat');
    sT = load('object2.mat');
    sT2 = load('object2.mat');
end
s0 = load('base.mat');
s1 = load('shoulder.mat');
s2 = load('upperarm.mat');
s3 = load('forearm.mat');
s4 = load('wrist1.mat');
s5 = load('wrist2.mat');
s6 = load('wrist3.mat');
if ee_opt == 1
    sSC = load('fts150_coupling.mat');
    sS = load('fts150.mat');
    sGC = load('gripper_coupling.mat');
    sGB = load('arg2f_85_base_link_simp.mat');
    sOK = load('arg2f_85_outer_knuckle_simp.mat');
    sOF = load('arg2f_85_outer_finger_simp.mat');
    sIK = load('arg2f_85_inner_knuckle_simp.mat');
    sIF = load('arg2f_85_inner_finger_pad_simp.mat');
end
if ee_opt == 2
    sH = load('head.mat');
    sGH = load('glove.mat');
end
sCF = load('coord_frame.mat');
sCFd = load('coord_frame.mat');

% Plot imported CAD data from file
animFig = figure('Name','Robot Plot');
animPlot = axes('Parent',animFig);

if feature == 1
    pB = patch(animPlot,'Faces', sB.modelstruct.F, 'Vertices' ,sB.modelstruct.V);
    set(pB, 'facec', 'flat');               % Set the face color flat
    set(pB, 'FaceColor', sB.modelstruct.C);	% Set the face color
    set(pB, 'EdgeColor', 'none');        	% Set the edge color

    pT = patch(animPlot,'Faces', sT.modelstruct.F, 'Vertices' ,sT.modelstruct.V);
    set(pT, 'facec', 'flat');               % Set the face color flat
    set(pT, 'FaceColor', sT.modelstruct.C);	% Set the face color
    set(pT, 'EdgeColor', 'none');        	% Set the edge color

    pT2 = patch(animPlot,'Faces', sT2.modelstruct.F, 'Vertices' ,sT2.modelstruct.V);
    set(pT2, 'facec', 'flat');               % Set the face color flat
    set(pT2, 'FaceColor', sT2.modelstruct.C);	% Set the face color
    set(pT2, 'EdgeColor', 'none');        	% Set the edge color
end

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

pCF = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCF, 'facec', 'flat');                      % Set the face color flat
set(pCF, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCF, 'EdgeColor', 'none');
%%

if TCP == 0
    tool_alpha = 0.05;
else
    tool_alpha = model_alpha;
end

[a_i,alpha_i,d_i,theta_i_O] = kinematicParametersUR3;

if feature == 1
    TI_Base = [rot_y(-pi/2),[0;0;0];zeros(1,3),1];
else
    TI_Base = eye(4);
end
TI_Base2 = [rot_x(0)*rot_z(0),[0.04;0.0;-0.85];zeros(1,3),1];
TI_B = eye(4);
TB_0 = eye(4);
TI_0 = TI_B*TB_0;
TI2_0 = [rot_x(0)*rot_z(pi)*rot_y(0),[0.0;0.0;0.0];zeros(1,3),1];
TI_i = zeros(4,4,6);
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

if TCP == 1
    dx_e = 0.000;
    dy_e = 0.030;
    dz_e = 0.140;
elseif TCP == 2
    dx_e = 0.000;
    dy_e = 0.000;
    dz_e = Tn_sC(3,4)+TsC_S(3,4)+TS_gC(3,4)+TgC_gB(3,4)+TgB_E(3,4);
else
    dx_e = 0.000;
    dy_e = 0.000;
    dz_e = 0;
end

Tn_E = [eye(3),[dx_e;dy_e;dz_e];zeros(1,3),1];

TB_Obj = [eye(3),Obj_Pos;zeros(1,3),1];
TI_Obj = TI_B*TB_Obj;

table1_offset = [-0.525;0.16;0.05];
TI_Table1 = [eye(3),Obj_Pos+table1_offset;zeros(1,3),1];
table2_offset = [0.6;0;0.27];
TI_Table2 = [eye(3),table2_offset;zeros(1,3),1];

% all
kinPara.a_i = a_i;
kinPara.alpha_i = alpha_i;
kinPara.d_i = d_i;
kinPara.theta_i_O = theta_i_O;
kinPara.d_e = dz_e;
kinPara.TI_0 = TI_0;

% task
q_posLim = [-360	,360
            -360	,360
         	-360    ,360
           	-360    ,360
           	-360    ,360
           	-360    ,360];

inv_geo_trn = 0;
kp_inv = 0.23;
kr_inv = 0.005;
kp_trn = 0.003;
kr_trn = 0.0001;
wp = 1;
wr = 0.1;
        
kinPara.q_posLim = q_posLim;
kinPara.inv_geo_trn = inv_geo_trn;
kinPara.kp_inv = kp_inv;
kinPara.kr_inv = kr_inv;
kinPara.kp_trn = kp_trn;
kinPara.kr_trn = kr_trn;
kinPara.wp = wp;
kinPara.wr = wr;
        

% trnsprt_num = size(Pos_transport,2);
trnsprt_num = length(transport_duration);

Vel = zeros(3,2);
Acc = zeros(3,2);

if mj_constrained == 1
    VelMJ = zeros(3,trnsprt_num+1);
    AccMJ = zeros(3,trnsprt_num+1);
else
    VelMJ = zeros(3,2);
    AccMJ = zeros(3,2);
end
    
point_num = sequence_num+trnsprt_num-1;

trnsprt_seq = 0;
for i = 1:sequence_num
    if mot_seq(i) == m_transport_high || mot_seq(i) == m_transport_low
        trnsprt_seq = trnsprt_seq + 1;
    end
end

Pos = zeros(3,sequence_num+trnsprt_num-1);
Pos(:,1) = Pos_ini;
T = zeros(1,sequence_num+trnsprt_num-1);
PosMJ = zeros(3,trnsprt_num+1);
TMJ = zeros(1,trnsprt_num+1);

Cj = zeros(6*3,sequence_num);
Coj = zeros(6*3,sequence_num);
Cj_trnsprt = zeros(6*6,sequence_num);
Coj_trnsprt = zeros(6*6,sequence_num);
for i = 2:sequence_num+1
    if mot_seq(i-1) == m_approach || mot_seq(i-1) == m_recede || mot_seq(i-1) == m_put || mot_seq(i-1) == m_pull
        if mot_seq(i-1) == m_approach
            Pos(:,i) = Pos(:,i-1) + [0;app_rec_dist;0];
            T(i) = T(i-1) + app_rec_duration;
        elseif mot_seq(i-1) == m_recede
            Pos(:,i) = Pos(:,i-1) - [0;app_rec_dist;0];
            T(i) = T(i-1) + app_rec_duration;
        elseif mot_seq(i-1) == m_put
            Pos(:,i) = Pos(:,i-1) - [hover_dist;0;0];
            T(i) = T(i-1) + put_duration;
        elseif mot_seq(i-1) == m_pull
            Pos(:,i) = Pos(:,i-1) + [hover_dist;0;0];
            T(i) = T(i-1) + put_duration;
        end
        Cj(:,i-1) = minimumJerkCoefficient(T(i-1:i)-T(i-1),Pos(:,i-1:i),Vel,Acc);
        Coj(:,i-1) = minimumJerkCoefficient(T(i-1:i)-T(i-1),Orn,Vel,Acc);
    elseif mot_seq(i-1) == m_transport_high || mot_seq(i-1) == m_transport_low
        PosMJ(:,1) = Pos(:,i-1);
        TMJ(1) = T(i-1);
        for k = 1:trnsprt_num
            if mot_seq(i-1) == m_transport_high
                PosMJ(:,k+1) = PosMJ(:,k) + Pos_transport(:,k);
            else
                PosMJ(:,k+1) = PosMJ(:,k) - Pos_transport(:,trnsprt_num-k+1);
            end
            TMJ(k+1) = TMJ(k) + transport_duration(k);
        end
        Pos(:,i) = PosMJ(:,trnsprt_num+1);
        T(i) = T(i-1) + sum(transport_duration);
        if mj_constrained == 1
            Cj_trnsprt(:,i-1) = minimumJerkConstrCoefficient(TMJ-TMJ(1),PosMJ,VelMJ,AccMJ);
            Coj_trnsprt(:,i-1) = minimumJerkConstrCoefficient(TMJ-TMJ(1),OrnMJ,VelMJ,AccMJ);
        else
            Cj_trnsprt(:,i-1) = minimumJerkCoefficient(TMJ-TMJ(1),PosMJ,VelMJ,AccMJ);
            Coj_trnsprt(:,i-1) = minimumJerkCoefficient(TMJ-TMJ(1),OrnMJ,VelMJ,AccMJ);
        end
    elseif mot_seq(i-1) == m_grab || mot_seq(i-1) == m_release
        Pos(:,i) = Pos(:,i-1);
        T(i) = T(i-1) + grab_rel_duration;
    end
end
T
Pos

% Cj
% Coj

total_step = length(T(1):tstp:T(point_num));

q_pos = zeros(6,total_step);
q_vel = zeros(6,total_step);
q_acc = zeros(6,total_step);
q_jer = zeros(6,total_step);

t_pos = zeros(3,total_step);
t_vel = zeros(3,total_step);
t_acc = zeros(3,total_step);
t_jer = zeros(3,total_step);
t_fbk = zeros(3,total_step);
t2_fbk = zeros(3,total_step);

r_pos = zeros(3,total_step);
r_vel = zeros(3,total_step);
r_acc = zeros(3,total_step);
r_jer = zeros(3,total_step);
r_fbk = zeros(3,total_step);

x_pos = zeros(6,total_step);
x2_pos = zeros(6,total_step);
tO_pos = zeros(3,total_step);

t = T(1);
t_inner = t;
stepcount = 1;

light(animPlot)             % add a default light
view(animPlot,[-150,25])	% Isometric view
grid(animPlot,'on')
xlabel(animPlot,'X'),ylabel(animPlot,'Y'),zlabel(animPlot,'Z')
set(animPlot,	'Projection','perspective', ...
                'PlotBoxAspectRatio',[1 1 1], ...
                'DataAspectRatio',[1 1 1]);

if feature == 1
    temp = [sB.modelstruct.V,ones(size(sB.modelstruct.V(:,1)))]*TI_Base2';
    set(pB,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)

    temp = [sT.modelstruct.V,ones(size(sT.modelstruct.V(:,1)))]*TI_Table1'*TI_Base2';
    set(pT,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)

    temp = [sT2.modelstruct.V,ones(size(sT2.modelstruct.V(:,1)))]*TI_Table2'*TI_Table1'*TI_Base2';
    set(pT2,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
end
    
temp = [s0.modelstruct.V,ones(size(s0.modelstruct.V(:,1)))]*TI_0'*TI_Base';
set(p0,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)

t_dist = 0;
for k = 1:total_step
    
    if t > T(stepcount + 1)
        stepcount = stepcount + 1;
        t_inner = 0;
    end
    
    cur_seq = mot_seq(stepcount);

    if (cur_seq == m_approach || cur_seq == m_put || cur_seq == m_recede || cur_seq == m_pull)
        [t_pos(:,k),t_vel(:,k),t_acc(:,k),t_jer(:,k)] = minimumJerkPolynomial(t_inner,T(stepcount:stepcount+1)-T(stepcount),Cj(:,stepcount));
        [r_pos(:,k),r_vel(:,k),r_acc(:,k),r_jer(:,k)] = minimumJerkPolynomial(t_inner,T(stepcount:stepcount+1)-T(stepcount),Coj(:,stepcount));
        if (cur_seq == m_put || cur_seq == m_pull)
            TI_Obj(1:3,4) = t_pos(:,k)+t_dist;
        end
    elseif (cur_seq == m_transport_high || cur_seq == m_transport_low)
        [t_pos(:,k),t_vel(:,k),t_acc(:,k),t_jer(:,k)] = minimumJerkPolynomial(t_inner,TMJ-TMJ(1),Cj_trnsprt(:,stepcount));
        [r_pos(:,k),r_vel(:,k),r_acc(:,k),r_jer(:,k)] = minimumJerkPolynomial(t_inner,TMJ-TMJ(1),Coj_trnsprt(:,stepcount));
        TI_Obj(1:3,4) = t_pos(:,k)+t_dist;
    else
        t_pos(:,k) = t_pos(:,k-1);
        t_vel(:,k) = t_vel(:,k-1);
        t_acc(:,k) = t_acc(:,k-1);
        t_jer(:,k) = t_jer(:,k-1);
        r_pos(:,k) = r_pos(:,k-1);
        r_vel(:,k) = r_vel(:,k-1);
        r_acc(:,k) = r_acc(:,k-1);
        r_jer(:,k) = r_jer(:,k-1);
        t_dist = TI_Obj(1:3,4)-t_pos(:,k);
    end

    t_inner = round(t_inner + tstp,3);
    t = round(t + tstp,3);
    
    % inverse kinematics
    RGoal = getRotMatfromEA(r_pos(:,k),euler_set);
    TGoal = [RGoal,t_pos(:,k);zeros(1,3),1];

    [q_pos(:,k),err] = invKinAnaBasic3(kinPara,TGoal,qPrev);
    if err == 1
        disp('error!')
        q_pos(:,k) = invGeoNumeric(kinPara,TGoal,qPrev);
    end
    % [q_pos(:,k),err] = invKinAnaBasic3(kinPara,TGoal,qPrev);
    % q_pos(:,k) = invGeoNumeric(kinPara,TGoal,qPrev);

    % norm(q_pos(:,k)-q_pos2(:,k))
    % if norm(q_pos(:,k)-q_pos2(:,k)) > 0.3
    %     TGoal
    %     q_pos(:,k)
    %     q_pos2(:,k)
    %     break;
    % end    
    % q_pos(:,k)
    qPrev = q_pos(:,k);

    % forward kinematics
    [T0_h,~] = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+q_pos(:,k));

    for i = 1:6
        TI_i(:,:,i) = TI_0*T0_h(:,:,i+1);
        % TI2_i(:,:,i) = TI2_0*T0_h(:,:,i+1);
    end

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

    % TI_Base'
    % TBase_I = [TI_Base(1:3,1:3)',-TI_Base(1:3,1:3)'*TI_Base(1:3,4);zeros(1,3),1];

    TI_E = TI_i(:,:,6)*Tn_E;
    TBase_E = TI_Base*TI_i(:,:,6)*Tn_E;
    % TI2_E = TI2_i(:,:,6);

    t_fbk(:,k) = TI_E(1:3,4);
    t2_fbk(:,k) = TBase_E(1:3,4);
    r_fbk(:,k) = getEulerPosVec(TI_E(1:3,1:3), euler_set);

    x_pos(:,k) = [t_fbk(:,k);r_fbk(:,k)];
    x2_pos(:,k) = [t2_fbk(:,k);r_fbk(:,k)];

    TBase_O = TI_Base*TI_Obj;
    tO_pos(:,k) = TBase_O(1:3,4);

    % transformation
    temp = [s1.modelstruct.V,ones(size(s1.modelstruct.V(:,1)))]*TI_i(:,:,1)'*TI_Base';
    set(p1,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s2.modelstruct.V,ones(size(s2.modelstruct.V(:,1)))]*TI_i(:,:,2)'*TI_Base';
    set(p2,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s3.modelstruct.V,ones(size(s3.modelstruct.V(:,1)))]*TI_i(:,:,3)'*TI_Base';
    set(p3,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s4.modelstruct.V,ones(size(s4.modelstruct.V(:,1)))]*TI_i(:,:,4)'*TI_Base';
    set(p4,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s5.modelstruct.V,ones(size(s5.modelstruct.V(:,1)))]*TI_i(:,:,5)'*TI_Base';
    set(p5,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s6.modelstruct.V,ones(size(s6.modelstruct.V(:,1)))]*TI_i(:,:,6)'*TI_Base';
    set(p6,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [sSC.modelstruct.V,ones(size(sSC.modelstruct.V(:,1)))]*TI_sC'*TI_Base';
    set(pSC,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sS.modelstruct.V,ones(size(sS.modelstruct.V(:,1)))]*TI_S'*TI_Base';
    set(pS,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sGC.modelstruct.V,ones(size(sGC.modelstruct.V(:,1)))]*TI_gC'*TI_Base';
    set(pGC,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sGB.modelstruct.V,ones(size(sGB.modelstruct.V(:,1)))]*TI_gB'*TI_Base';
    set(pGB,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKa'*TI_Base';
    set(pOKa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKb'*TI_Base';
    set(pOKb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFa'*TI_Base';
    set(pOFa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFb'*TI_Base';
    set(pOFb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKa'*TI_Base';
    set(pIKa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKb'*TI_Base';
    set(pIKb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFa'*TI_Base';
    set(pIFa,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFb'*TI_Base';
    set(pIFb,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_Obj'*TI_Base';
    set(pCF,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    
    hold(animPlot,'on')
    grid(animPlot,'on')
    % plot3(animPlot,x_pos(1,1:k),x_pos(2,1:k),x_pos(3,1:k),'b')
    plot3(animPlot,x2_pos(1,1:k),x2_pos(2,1:k),x2_pos(3,1:k),'b','LineWidth',1)
    plot3(animPlot,tO_pos(1,1:k),tO_pos(2,1:k),tO_pos(3,1:k),'r','LineWidth',2)
    hold(animPlot,'off')
    if write_on == 0
        drawnow
    end
    
end

figure
tiledlayout(3,1)
ax1 = nexttile;
plot(0:tstp:tstp*(total_step-1),t_pos(1,:))
title(ax1,"x-axis")
ax2 = nexttile;
plot(0:tstp:tstp*(total_step-1),t_pos(2,:))
title(ax2,"y-axis")
ax3 = nexttile;
plot(0:tstp:tstp*(total_step-1),t_pos(3,:))
title(ax3,"z-axis")

if write_on == 1
    if write_task == 0
        q_pos_write = q_pos';
        writematrix(q_pos_write,'q_posCmd_131123.csv')
    else
        % x_temp = x_pos';
        % x_temp(:,1) = -x_temp(:,1);
        % x_temp(:,2) = -x_temp(:,2)+0.19;
        % x_temp(:,4:6) = ones(size(x_temp(:,4:6))).*[1.199626905,-1.207793505,1.204287902];
        % x_pos_write = x_temp;
        x_pos_write = x2_pos';
        writematrix(x_pos_write,'x_posCmd_131123.csv')
    end
end

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