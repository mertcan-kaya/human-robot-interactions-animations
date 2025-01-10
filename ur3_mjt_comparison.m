clc, clear all, close all

% 0: Base, 1: View, 2: Tool
feature = 0;

% 0: TCP, 1: TCP_1, 2: TCP_2
TCP = 0;

% solidness: (0,1)
model_alpha = 1;

% 0: static, 1: joint, 2: task
motion = 1;

if motion == 2
    
    T = [0 4 6 10 14 16 20];

    Pos = [ -0.2340	0.4000  0.4000	0.4000	0.4000  0.4000  0.3900      % x-axis
            0.5341	0.3500  0.3500  0.3500  0.3500  0.3500  0.2500      % y-axis
            0.4483  0.9600  0.8800  0.8800  0.8800  0.9600  0.9500];	% z-axis
        
    Pos = [ -0.2340	-0.2840 -0.2840	-0.2840	-0.2840 -0.2840 -0.2340      % x-axis
            0.5341	0.4341  0.5341  0.4341  0.5341  0.4341  0.5341      % y-axis
            0.4483  0.3483  0.4483  0.3483  0.4483  0.3483  0.4483];	% z-axis

    euler_set = 1; % 1: ZYZ, 2: ZYX, 3: XYZ, 4:ZXZ

    Orn = deg2rad([ 90	90	90	90	90	90   90     % phi-axis
                    90	90	90	90	90	90  90     % theta-axis
                    180	180	180	180	180	180  180]);	% psi-axis
                
    Vel = [ 0 0
            0 0
            0 0];

    Acc = [ 0 0
            0 0
            0 0];

elseif motion == 1
    
    % direction of motion: 0 is horizontal, 1 is vertical
    direction = 1;
    
    % 0: minimum-jerk, 1: trapezoidal
    trj_type = 0;
    
    mj_constrained = 1;
    trapez_ratio = 8;

    T = [0 4 10 16 22 28 32];
    
    if direction == 1
        Pos = deg2rad([ 90 90 90 90 90 90 90
                        0 60 -60 60 -60 60 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        90 90 90 90 90 90 90
                        90 90 90 90 90 90 90]);
        Pos = deg2rad([ 90 150 70 10 120 50 90
                        0 60 -60 60 -60 60 0
                        0 0 10 0 -10 0 0
                        0 50 0 0 50 0 0
                        90 0 90 -90 90 -90 90
                        90 120 90 30 90 -30 90]);
    else
        Pos = deg2rad([ 90 150 30 150 30 150 90
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        90 90 90 90 90 90 90
                        90 90 90 90 90 90 90]);
    end

%     if mj_constrained == 1
        VelC = deg2rad([ 0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0]);

        AccC = deg2rad([ 0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0
                        0 0 0 0 0 0 0]);
%     else
        Vel = deg2rad([ 0 0
                        0 0
                        0 0
                        0 0
                        0 0
                        0 0]);

        Acc = deg2rad([ 0 0
                        0 0
                        0 0
                        0 0
                        0 0
                        0 0]);
%     end

else
    q_pos = deg2rad([115;15;65;10;135;15]);
end

tstp = 0.1;

addpath('./funcs');
addpath('./mat');

s0 = load('base.mat');
s1 = load('shoulder.mat');
s2 = load('upperarm.mat');
s3 = load('forearm.mat');
s4 = load('wrist1.mat');
s5 = load('wrist2.mat');
s6 = load('wrist3.mat');
sSC = load('fts150_coupling.mat');
sS = load('fts150.mat');
sGC = load('gripper_coupling.mat');
sGB = load('arg2f_85_base_link_simp.mat');
sOK = load('arg2f_85_outer_knuckle_simp.mat');
sOF = load('arg2f_85_outer_finger_simp.mat');
sIK = load('arg2f_85_inner_knuckle_simp.mat');
sIF = load('arg2f_85_inner_finger_pad_simp.mat');
sCF = load('coord_frame');

% Plot imported CAD data from file
animFig1 = figure('Name','Robot Plot 1');
animPlot1 = axes('Parent',animFig1);
animFig1.Position = [125 120 850 750];

p01 = patch(animPlot1,'Faces', s0.modelstruct.F, 'Vertices' ,s0.modelstruct.V);
set(p01, 'facec', 'flat');               % Set the face color flat
set(p01, 'FaceColor', s0.modelstruct.C);	% Set the face color
set(p01, 'EdgeColor', 'none');        	% Set the edge color

p11 = patch(animPlot1,'Faces', s1.modelstruct.F, 'Vertices' ,s1.modelstruct.V);
set(p11, 'facec', 'flat');               % Set the face color flat
set(p11, 'FaceVertexCData', s1.modelstruct.C);	% Set the face color
set(p11, 'EdgeColor', 'none');        	% Set the edge color

p21 = patch(animPlot1,'Faces', s2.modelstruct.F, 'Vertices' ,s2.modelstruct.V);
set(p21, 'facec', 'flat');               % Set the face color flat
set(p21, 'FaceVertexCData', s2.modelstruct.C);	% Set the face color
set(p21, 'EdgeColor', 'none');        	% Set the edge color

p31 = patch(animPlot1,'Faces', s3.modelstruct.F, 'Vertices' ,s3.modelstruct.V);
set(p31, 'facec', 'flat');               % Set the face color flat
set(p31, 'FaceVertexCData', s3.modelstruct.C);	% Set the face color
set(p31, 'EdgeColor', 'none');        	% Set the edge color

p41 = patch(animPlot1,'Faces', s4.modelstruct.F, 'Vertices' ,s4.modelstruct.V);
set(p41, 'facec', 'flat');               % Set the face color flat
set(p41, 'FaceVertexCData', s4.modelstruct.C);	% Set the face color
set(p41, 'EdgeColor', 'none');        	% Set the edge color

p51 = patch(animPlot1,'Faces', s5.modelstruct.F, 'Vertices' ,s5.modelstruct.V);
set(p51, 'facec', 'flat');               % Set the face color flat
set(p51, 'FaceVertexCData', s5.modelstruct.C);	% Set the face color
set(p51, 'EdgeColor', 'none');        	% Set the edge color

p61 = patch(animPlot1,'Faces', s6.modelstruct.F, 'Vertices' ,s6.modelstruct.V);
set(p61, 'facec', 'flat');               % Set the face color flat
set(p61, 'FaceVertexCData', s6.modelstruct.C);	% Set the face color
set(p61, 'EdgeColor', 'none');        	% Set the edge color

% pSC1 = patch(animPlot1,'Faces', sSC.modelstruct.F, 'Vertices' ,sSC.modelstruct.V);
% set(pSC1, 'facec', 'flat');               % Set the face color flat
% set(pSC1, 'FaceVertexCData', sSC.modelstruct.C);	% Set the face color
% set(pSC1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pS1 = patch(animPlot1,'Faces', sS.modelstruct.F, 'Vertices' ,sS.modelstruct.V);
% set(pS1, 'facec', 'flat');               % Set the face color flat
% set(pS1, 'FaceVertexCData', sS.modelstruct.C);	% Set the face color
% set(pS1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pGC1 = patch(animPlot1,'Faces', sGC.modelstruct.F, 'Vertices' ,sGC.modelstruct.V);
% set(pGC1, 'facec', 'flat');               % Set the face color flat
% set(pGC1, 'FaceColor', sGC.modelstruct.C);	% Set the face color
% set(pGC1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pGB1 = patch(animPlot1,'Faces', sGB.modelstruct.F, 'Vertices' ,sGB.modelstruct.V);
% set(pGB1, 'facec', 'flat');               % Set the face color flat
% set(pGB1, 'FaceVertexCData', sGB.modelstruct.C);	% Set the face color
% set(pGB1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOKa1 = patch(animPlot1,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
% set(pOKa1, 'facec', 'flat');               % Set the face color flat
% set(pOKa1, 'FaceColor', sOK.modelstruct.C);	% Set the face color
% set(pOKa1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOKb1 = patch(animPlot1,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
% set(pOKb1, 'facec', 'flat');               % Set the face color flat
% set(pOKb1, 'FaceColor', sOK.modelstruct.C);	% Set the face color
% set(pOKb1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOFa1 = patch(animPlot1,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
% set(pOFa1, 'facec', 'flat');               % Set the face color flat
% set(pOFa1, 'FaceColor', sOF.modelstruct.C);	% Set the face color
% set(pOFa1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOFb1 = patch(animPlot1,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
% set(pOFb1, 'facec', 'flat');               % Set the face color flat
% set(pOFb1, 'FaceColor', sOF.modelstruct.C);	% Set the face color
% set(pOFb1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIKa1 = patch(animPlot1,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
% set(pIKa1, 'facec', 'flat');               % Set the face color flat
% set(pIKa1, 'FaceColor', sIK.modelstruct.C);	% Set the face color
% set(pIKa1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIKb1 = patch(animPlot1,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
% set(pIKb1, 'facec', 'flat');               % Set the face color flat
% set(pIKb1, 'FaceColor', sIK.modelstruct.C);	% Set the face color
% set(pIKb1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIFa1 = patch(animPlot1,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
% set(pIFa1, 'facec', 'flat');               % Set the face color flat
% set(pIFa1, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
% set(pIFa1, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIFb1 = patch(animPlot1,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
% set(pIFb1, 'facec', 'flat');               % Set the face color flat
% set(pIFb1, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
% set(pIFb1, 'EdgeColor', 'none');        	% Set the edge color

pCF1 = patch(animPlot1,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCF1, 'facec', 'flat');                      % Set the face color flat
set(pCF1, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCF1, 'EdgeColor', 'none');

% Plot imported CAD data from file
animFig2 = figure('Name','Robot Plot 2');
animPlot2 = axes('Parent',animFig2);
animFig2.Position = [975 120 850 750];

p02 = patch(animPlot2,'Faces', s0.modelstruct.F, 'Vertices' ,s0.modelstruct.V);
set(p02, 'facec', 'flat');               % Set the face color flat
set(p02, 'FaceColor', s0.modelstruct.C);	% Set the face color
set(p02, 'EdgeColor', 'none');        	% Set the edge color

p12 = patch(animPlot2,'Faces', s1.modelstruct.F, 'Vertices' ,s1.modelstruct.V);
set(p12, 'facec', 'flat');               % Set the face color flat
set(p12, 'FaceVertexCData', s1.modelstruct.C);	% Set the face color
set(p12, 'EdgeColor', 'none');        	% Set the edge color

p22 = patch(animPlot2,'Faces', s2.modelstruct.F, 'Vertices' ,s2.modelstruct.V);
set(p22, 'facec', 'flat');               % Set the face color flat
set(p22, 'FaceVertexCData', s2.modelstruct.C);	% Set the face color
set(p22, 'EdgeColor', 'none');        	% Set the edge color

p32 = patch(animPlot2,'Faces', s3.modelstruct.F, 'Vertices' ,s3.modelstruct.V);
set(p32, 'facec', 'flat');               % Set the face color flat
set(p32, 'FaceVertexCData', s3.modelstruct.C);	% Set the face color
set(p32, 'EdgeColor', 'none');        	% Set the edge color

p42 = patch(animPlot2,'Faces', s4.modelstruct.F, 'Vertices' ,s4.modelstruct.V);
set(p42, 'facec', 'flat');               % Set the face color flat
set(p42, 'FaceVertexCData', s4.modelstruct.C);	% Set the face color
set(p42, 'EdgeColor', 'none');        	% Set the edge color

p52 = patch(animPlot2,'Faces', s5.modelstruct.F, 'Vertices' ,s5.modelstruct.V);
set(p52, 'facec', 'flat');               % Set the face color flat
set(p52, 'FaceVertexCData', s5.modelstruct.C);	% Set the face color
set(p52, 'EdgeColor', 'none');        	% Set the edge color

p62 = patch(animPlot2,'Faces', s6.modelstruct.F, 'Vertices' ,s6.modelstruct.V);
set(p62, 'facec', 'flat');               % Set the face color flat
set(p62, 'FaceVertexCData', s6.modelstruct.C);	% Set the face color
set(p62, 'EdgeColor', 'none');        	% Set the edge color

% pSC2 = patch(animPlot2,'Faces', sSC.modelstruct.F, 'Vertices' ,sSC.modelstruct.V);
% set(pSC2, 'facec', 'flat');               % Set the face color flat
% set(pSC2, 'FaceVertexCData', sSC.modelstruct.C);	% Set the face color
% set(pSC2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pS2 = patch(animPlot2,'Faces', sS.modelstruct.F, 'Vertices' ,sS.modelstruct.V);
% set(pS2, 'facec', 'flat');               % Set the face color flat
% set(pS2, 'FaceVertexCData', sS.modelstruct.C);	% Set the face color
% set(pS2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pGC2 = patch(animPlot2,'Faces', sGC.modelstruct.F, 'Vertices' ,sGC.modelstruct.V);
% set(pGC2, 'facec', 'flat');               % Set the face color flat
% set(pGC2, 'FaceColor', sGC.modelstruct.C);	% Set the face color
% set(pGC2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pGB2 = patch(animPlot2,'Faces', sGB.modelstruct.F, 'Vertices' ,sGB.modelstruct.V);
% set(pGB2, 'facec', 'flat');               % Set the face color flat
% set(pGB2, 'FaceVertexCData', sGB.modelstruct.C);	% Set the face color
% set(pGB2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOKa2 = patch(animPlot2,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
% set(pOKa2, 'facec', 'flat');               % Set the face color flat
% set(pOKa2, 'FaceColor', sOK.modelstruct.C);	% Set the face color
% set(pOKa2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOKb2 = patch(animPlot2,'Faces', sOK.modelstruct.F, 'Vertices' ,sOK.modelstruct.V);
% set(pOKb2, 'facec', 'flat');               % Set the face color flat
% set(pOKb2, 'FaceColor', sOK.modelstruct.C);	% Set the face color
% set(pOKb2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOFa2 = patch(animPlot2,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
% set(pOFa2, 'facec', 'flat');               % Set the face color flat
% set(pOFa2, 'FaceColor', sOF.modelstruct.C);	% Set the face color
% set(pOFa2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pOFb2 = patch(animPlot2,'Faces', sOF.modelstruct.F, 'Vertices' ,sOF.modelstruct.V);
% set(pOFb2, 'facec', 'flat');               % Set the face color flat
% set(pOFb2, 'FaceColor', sOF.modelstruct.C);	% Set the face color
% set(pOFb2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIKa2 = patch(animPlot2,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
% set(pIKa2, 'facec', 'flat');               % Set the face color flat
% set(pIKa2, 'FaceColor', sIK.modelstruct.C);	% Set the face color
% set(pIKa2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIKb2 = patch(animPlot2,'Faces', sIK.modelstruct.F, 'Vertices' ,sIK.modelstruct.V);
% set(pIKb2, 'facec', 'flat');               % Set the face color flat
% set(pIKb2, 'FaceColor', sIK.modelstruct.C);	% Set the face color
% set(pIKb2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIFa2 = patch(animPlot2,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
% set(pIFa2, 'facec', 'flat');               % Set the face color flat
% set(pIFa2, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
% set(pIFa2, 'EdgeColor', 'none');        	% Set the edge color
% 
% pIFb2 = patch(animPlot2,'Faces', sIF.modelstruct.F, 'Vertices' ,sIF.modelstruct.V);
% set(pIFb2, 'facec', 'flat');               % Set the face color flat
% set(pIFb2, 'FaceVertexCData', sIF.modelstruct.C);	% Set the face color
% set(pIFb2, 'EdgeColor', 'none');        	% Set the edge color

pCF2 = patch(animPlot2,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCF2, 'facec', 'flat');                      % Set the face color flat
set(pCF2, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCF2, 'EdgeColor', 'none');

if TCP == 0
    tool_alpha = 0.05;
else
    tool_alpha = model_alpha;
end

[a_i,alpha_i,d_i,theta_i_O] = kinematicParametersUR3;

if feature == 1
    TI_B = [rot_x(-pi/2)*rot_z(pi/2),[0;-0.4;0];zeros(1,3),1];
else
    TI_B = eye(4);
end
% TB_0 = [rot_z(pi),[-0.00225;-0.0015;0];zeros(1,3),1];
% TB_0 = [rot_z(pi),[-0.00225;0;0];zeros(1,3),1];
TB_0 = [rot_z(pi),zeros(3,1);zeros(1,3),1];
TI_0 = TI_B*TB_0;
TI_i1 = zeros(4,4,6);
TI_i2 = zeros(4,4,6);
% Tn_sC   = [eye(3),[0;0;0.0085];zeros(1,3),1];
% TsC_S   = [eye(3),[0;0;0.0375];zeros(1,3),1];
% TS_gC   = [eye(3),[0;0;0.0111];zeros(1,3),1];
% TgC_gB  = [eye(3),[0;0;0.0900];zeros(1,3),1];
% TgB_E   = [eye(3),[0;0;0.0403];zeros(1,3),1];
% TgB_oKa = [rot_y(pi/4),[0.035;0;-0.035];zeros(1,3),1];
% TgB_oKb = [rot_z(pi)*rot_y(pi/4),[-0.035;0;-0.035];zeros(1,3),1];
% TgB_oFa = [rot_y(pi/2),[0.06;0;-0.015];zeros(1,3),1];
% TgB_oFb = [rot_z(pi)*rot_y(pi/2),[-0.06;0;-0.015];zeros(1,3),1];
% TgB_iKa = [rot_y(pi/2.25),[0.02;0;-0.025];zeros(1,3),1];
% TgB_iKb = [rot_z(pi)*rot_y(pi/2.25),[-0.02;0;-0.025];zeros(1,3),1];
% TgB_iFa = [rot_y(pi/2),[0.065;0;0.035];zeros(1,3),1];
% TgB_iFb = [rot_z(pi)*rot_y(pi/2),[-0.065;0;0.035];zeros(1,3),1];

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
        
if motion ~= 0
    
    n = length(T)-1;  % n= number of trajectories 
    
    if motion == 2
        if mj_constrained == 1
            Cj = minimumJerkConstrCoefficient(T,Pos,Vel,Acc);
            Coj = minimumJerkConstrCoefficient(T,Orn,Vel,Acc);
        else
            Cj = minimumJerkCoefficient(T,Pos,Vel,Acc);
            Coj = minimumJerkCoefficient(T,Orn,Vel,Acc);
        end
        
        qPrev = zeros(6,1);
    else
        CjC = zeros(6*n,6);
        Cj = zeros(6*n,6);
        for j = 1:6
%             if mj_constrained == 1
                CjC(:,j) = minimumJerkConstrCoefficient1DOF(T,Pos(j,:),VelC(j,:),AccC(j,:));
%             else
                Cj(:,j) = minimumJerkCoefficient1DOF(T,Pos(j,:),Vel(j,:),Acc(j,:));
%             end
        end
    end

    total_step = length(T(1):tstp:T(n+1));

    t = T(1);

    q_pos1 = zeros(6,total_step);
    q_vel1 = zeros(6,total_step);
    q_acc1 = zeros(6,total_step);
    q_jer1 = zeros(6,total_step);

    q_pos2 = zeros(6,total_step);
    q_vel2 = zeros(6,total_step);
    q_acc2 = zeros(6,total_step);
    q_jer2 = zeros(6,total_step);

    q_pos3 = zeros(6,total_step);
    q_vel3 = zeros(6,total_step);
    q_acc3 = zeros(6,total_step);
    q_jer3 = zeros(6,total_step);

    t_pos = zeros(3,total_step);
    t_vel = zeros(3,total_step);
    t_acc = zeros(3,total_step);
    t_jer = zeros(3,total_step);
    t_fbk1 = zeros(3,total_step);
    t_fbk2 = zeros(3,total_step);

    r_pos = zeros(3,total_step);
    r_vel = zeros(3,total_step);
    r_acc = zeros(3,total_step);
    r_jer = zeros(3,total_step);
    r_fbk = zeros(3,total_step);

    x_pos = zeros(3,total_step);
else
    total_step = 1;
end

light(animPlot1)             % add a default light
view(animPlot1,[-225,30])	% Isometric view
grid(animPlot1,'on')
xlabel(animPlot1,'X'),ylabel(animPlot1,'Y'),zlabel(animPlot1,'Z')
set(animPlot1,	'Projection','perspective', ...
                'PlotBoxAspectRatio',[1 1 1], ...
                'DataAspectRatio',[1 1 1]);
% axis(animPlot1,[-0.1 0.65 -0.75 0.3 -0.4 0.3]);
axis(animPlot1,[-0.2 0.65 -0.7 0.25 -0.5 0.55]);

temp = [s0.modelstruct.V,ones(size(s0.modelstruct.V(:,1)))]*TI_0';
set(p01,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)

light(animPlot2)             % add a default light
view(animPlot2,[-225,30])	% Isometric view
grid(animPlot2,'on')
xlabel(animPlot2,'X'),ylabel(animPlot2,'Y'),zlabel(animPlot2,'Z')
set(animPlot2,	'Projection','perspective', ...
                'PlotBoxAspectRatio',[1 1 1], ...
                'DataAspectRatio',[1 1 1]);
axis(animPlot2,[-0.2 0.65 -0.7 0.25 -0.5 0.55]);

temp = [s0.modelstruct.V,ones(size(s0.modelstruct.V(:,1)))]*TI_0';
set(p02,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)

for k = 1:total_step
    
    if motion ~= 0
        
        if motion == 2
            
            [t_pos(:,k),t_vel(:,k),t_acc(:,k),t_jer(:,k)] = minimumJerkPolynomial(t,T,Cj);
            [r_pos(:,k),r_vel(:,k),r_acc(:,k),r_jer(:,k)] = minimumJerkPolynomial(t,T,Coj);
    
            RGoal = getRotMatfromEA(r_pos(:,k),euler_set);
            TGoal = [RGoal,t_pos(:,k);zeros(1,3),1];
    
            [q_pos1(:,k),err] = invKinAnaBasic(kinPara,TGoal,qPrev);
            if err == 1
                q_pos1(:,k) = invGeoNumeric(kinPara,TGoal,qPrev);
            end
            
            qPrev = q_pos1(:,k);
        else
            for j = 1:6
                [q_pos1(j,k),q_vel1(j,k),q_acc1(j,k),q_jer1(j,k)] = p2pTrjTrap1DOF(t,T,Pos(j,:),trapez_ratio);
                [q_pos2(j,k),q_vel2(j,k),q_acc2(j,k),q_jer2(j,k)] = minimumJerkPolynomial1DOF(t,T,Cj(:,j));
                [q_pos3(j,k),q_vel3(j,k),q_acc3(j,k),q_jer3(j,k)] = minimumJerkPolynomial1DOF(t,T,CjC(:,j));
            end
%             rad2deg(q_pos(:,k))
        end
        
    end
    
    [T0_h1,~] = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+q_pos1(:,k));
    [T0_h2,~] = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+q_pos2(:,k));

    for i = 1:6
        TI_i1(:,:,i) = TI_0*T0_h1(:,:,i+1);
        TI_i2(:,:,i) = TI_0*T0_h2(:,:,i+1);
    end

%     TI_sC1 = TI_i1(:,:,6)*Tn_sC;
%     TI_S1 = TI_sC1*TsC_S;
%     TI_gC1 = TI_S1*TS_gC;
%     TI_gB1 = TI_gC1*TgC_gB;
%     TI_oKa1 = TI_gB1*TgB_oKa;
%     TI_oKb1 = TI_gB1*TgB_oKb;
%     TI_oFa1 = TI_gB1*TgB_oFa;
%     TI_oFb1 = TI_gB1*TgB_oFb;
%     TI_iKa1 = TI_gB1*TgB_iKa;
%     TI_iKb1 = TI_gB1*TgB_iKb;
%     TI_iFa1 = TI_gB1*TgB_iFa;
%     TI_iFb1 = TI_gB1*TgB_iFb;
% 
%     TI_sC2 = TI_i2(:,:,6)*Tn_sC;
%     TI_S2 = TI_sC2*TsC_S;
%     TI_gC2 = TI_S2*TS_gC;
%     TI_gB2 = TI_gC2*TgC_gB;
%     TI_oKa2 = TI_gB2*TgB_oKa;
%     TI_oKb2 = TI_gB2*TgB_oKb;
%     TI_oFa2 = TI_gB2*TgB_oFa;
%     TI_oFb2 = TI_gB2*TgB_oFb;
%     TI_iKa2 = TI_gB2*TgB_iKa;
%     TI_iKb2 = TI_gB2*TgB_iKb;
%     TI_iFa2 = TI_gB2*TgB_iFa;
%     TI_iFb2 = TI_gB2*TgB_iFb;
    
    TI_E1 = TI_i1(:,:,6)*Tn_E;
    TI_E2 = TI_i2(:,:,6)*Tn_E;
    % TI_E(1:3,4)

    % t_fbk

    % q_pos
    % q = invKinAnalytic(kinPara,TI_E,q_pos);

    % transformation
    temp = [s1.modelstruct.V,ones(size(s1.modelstruct.V(:,1)))]*TI_i1(:,:,1)';
    set(p11,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s2.modelstruct.V,ones(size(s2.modelstruct.V(:,1)))]*TI_i1(:,:,2)';
    set(p21,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s3.modelstruct.V,ones(size(s3.modelstruct.V(:,1)))]*TI_i1(:,:,3)';
    set(p31,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s4.modelstruct.V,ones(size(s4.modelstruct.V(:,1)))]*TI_i1(:,:,4)';
    set(p41,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s5.modelstruct.V,ones(size(s5.modelstruct.V(:,1)))]*TI_i1(:,:,5)';
    set(p51,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s6.modelstruct.V,ones(size(s6.modelstruct.V(:,1)))]*TI_i1(:,:,6)';
    set(p61,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
%     temp = [sSC.modelstruct.V,ones(size(sSC.modelstruct.V(:,1)))]*TI_sC1';
%     set(pSC1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sS.modelstruct.V,ones(size(sS.modelstruct.V(:,1)))]*TI_S1';
%     set(pS1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sGC.modelstruct.V,ones(size(sGC.modelstruct.V(:,1)))]*TI_gC1';
%     set(pGC1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sGB.modelstruct.V,ones(size(sGB.modelstruct.V(:,1)))]*TI_gB1';
%     set(pGB1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKa1';
%     set(pOKa1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKb1';
%     set(pOKb1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFa1';
%     set(pOFa1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFb1';
%     set(pOFb1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKa1';
%     set(pIKa1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKb1';
%     set(pIKb1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFa1';
%     set(pIFa1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFb1';
%     set(pIFb1,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_E1';
    set(pCF1,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    
    
    temp = [s1.modelstruct.V,ones(size(s1.modelstruct.V(:,1)))]*TI_i2(:,:,1)';
    set(p12,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s2.modelstruct.V,ones(size(s2.modelstruct.V(:,1)))]*TI_i2(:,:,2)';
    set(p22,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s3.modelstruct.V,ones(size(s3.modelstruct.V(:,1)))]*TI_i2(:,:,3)';
    set(p32,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s4.modelstruct.V,ones(size(s4.modelstruct.V(:,1)))]*TI_i2(:,:,4)';
    set(p42,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s5.modelstruct.V,ones(size(s5.modelstruct.V(:,1)))]*TI_i2(:,:,5)';
    set(p52,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    temp = [s6.modelstruct.V,ones(size(s6.modelstruct.V(:,1)))]*TI_i2(:,:,6)';
    set(p62,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
%     temp = [sSC.modelstruct.V,ones(size(sSC.modelstruct.V(:,1)))]*TI_sC2';
%     set(pSC2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sS.modelstruct.V,ones(size(sS.modelstruct.V(:,1)))]*TI_S2';
%     set(pS2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sGC.modelstruct.V,ones(size(sGC.modelstruct.V(:,1)))]*TI_gC2';
%     set(pGC2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sGB.modelstruct.V,ones(size(sGB.modelstruct.V(:,1)))]*TI_gB2';
%     set(pGB2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKa2';
%     set(pOKa2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKb2';
%     set(pOKb2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFa2';
%     set(pOFa2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFb2';
%     set(pOFb2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKa2';
%     set(pIKa2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKb2';
%     set(pIKb2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFa2';
%     set(pIFa2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
%     temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFb2';
%     set(pIFb2,'Vertices',temp(:,1:3),'FaceAlpha',tool_alpha)
    temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_E2';
    set(pCF2,'Vertices',temp(:,1:3),'FaceAlpha',model_alpha)
    
    if motion ~= 0
        t = round(t + tstp,3);
        
        hold(animPlot1,'on')
        grid(animPlot1,'on')
        plot3(animPlot1,x_pos(1,1:k),x_pos(2,1:k),x_pos(3,1:k),'b')
        hold(animPlot1,'off')
        drawnow
    end
    
    if model_alpha < 0.5
        hold(animPlot1,'on')
        plot3(animPlot1,TI_E(1,4),TI_E(2,4),TI_E(3,4),'b*')
        plot3(animPlot1,TI_i(1,4,6),TI_i(2,4,6),TI_i(3,4,6),'r*')
        plot3(animPlot1,TI_i(1,4,5),TI_i(2,4,5),TI_i(3,4,5),'g*')
        plot3(animPlot1,TI_i(1,4,4),TI_i(2,4,4),TI_i(3,4,4),'m*')
        plot3(animPlot1,TI_i(1,4,3),TI_i(2,4,3),TI_i(3,4,3),'c*')
        plot3(animPlot1,TI_i(1,4,2),TI_i(2,4,2),TI_i(3,4,2),'k*')
        hold(animPlot1,'off')
    end
    
end

jntPosFig = figure('Name','Joint Pos Data Plot');
jntPosFig.Position = [450 120 1050 750];
pos1ax = subplot(3,2,1);
hold(pos1ax,'on')
plot(pos1ax,T(1):tstp:T(n+1),rad2deg(q_pos1(1,1:end)))
plot(pos1ax,T(1):tstp:T(n+1),rad2deg(q_pos2(1,1:end)))
plot(pos1ax,T(1):tstp:T(n+1),rad2deg(q_pos3(1,1:end)))
plot(pos1ax,T,rad2deg(Pos(1,:)),'k.')
hold(pos1ax,'off')
grid(pos1ax,'on')
xlabel(pos1ax,'Time (sec)')
ylabel(pos1ax,'Position (deg)');
title(pos1ax,'joint 1')
xlim(pos1ax,[T(1) T(n+1)]);
pos2ax = subplot(3,2,2);
hold(pos2ax,'on')
plot(pos2ax,T(1):tstp:T(n+1),rad2deg(q_pos1(2,1:end)))
plot(pos2ax,T(1):tstp:T(n+1),rad2deg(q_pos2(2,1:end)))
plot(pos2ax,T(1):tstp:T(n+1),rad2deg(q_pos3(2,1:end)))
plot(pos2ax,T,rad2deg(Pos(2,:)),'k.')
hold(pos2ax,'off')
grid(pos2ax,'on')
xlabel(pos2ax,'Time (sec)')
ylabel(pos2ax,'Position (deg)');
title(pos2ax,'joint 2')
xlim(pos2ax,[T(1) T(n+1)]);
pos3ax = subplot(3,2,3);
hold(pos3ax,'on')
plot(pos3ax,T(1):tstp:T(n+1),rad2deg(q_pos1(3,1:end)))
plot(pos3ax,T(1):tstp:T(n+1),rad2deg(q_pos2(3,1:end)))
plot(pos3ax,T(1):tstp:T(n+1),rad2deg(q_pos3(3,1:end)))
plot(pos3ax,T,rad2deg(Pos(3,:)),'k.')
hold(pos3ax,'off')
grid(pos3ax,'on')
xlabel(pos3ax,'Time (sec)')
ylabel(pos3ax,'Position (deg)');
title(pos3ax,'joint 3')
xlim(pos3ax,[T(1) T(n+1)]);
pos4ax = subplot(3,2,4);
hold(pos4ax,'on')
plot(pos4ax,T(1):tstp:T(n+1),rad2deg(q_pos1(4,1:end)))
plot(pos4ax,T(1):tstp:T(n+1),rad2deg(q_pos2(4,1:end)))
plot(pos4ax,T(1):tstp:T(n+1),rad2deg(q_pos3(4,1:end)))
plot(pos4ax,T,rad2deg(Pos(4,:)),'k.')
hold(pos4ax,'off')
grid(pos4ax,'on')
xlabel(pos4ax,'Time (sec)')
ylabel(pos4ax,'Position (deg)');
title(pos4ax,'joint 4')
xlim(pos4ax,[T(1) T(n+1)]);
pos5ax = subplot(3,2,5);
hold(pos5ax,'on')
plot(pos5ax,T(1):tstp:T(n+1),rad2deg(q_pos1(5,1:end)))
plot(pos5ax,T(1):tstp:T(n+1),rad2deg(q_pos2(5,1:end)))
plot(pos5ax,T(1):tstp:T(n+1),rad2deg(q_pos3(5,1:end)))
plot(pos5ax,T,rad2deg(Pos(5,:)),'k.')
hold(pos5ax,'off')
grid(pos5ax,'on')
xlabel(pos5ax,'Time (sec)')
ylabel(pos5ax,'Position (deg)');
title(pos5ax,'joint 5')
xlim(pos5ax,[T(1) T(n+1)]);
pos6ax = subplot(3,2,6);
hold(pos6ax,'on')
plot(pos6ax,T(1):tstp:T(n+1),rad2deg(q_pos1(6,1:end)))
plot(pos6ax,T(1):tstp:T(n+1),rad2deg(q_pos2(6,1:end)))
plot(pos6ax,T(1):tstp:T(n+1),rad2deg(q_pos3(6,1:end)))
plot(pos6ax,T,rad2deg(Pos(6,:)),'k.')
hold(pos6ax,'off')
grid(pos6ax,'on')
xlabel(pos6ax,'Time (sec)')
ylabel(pos6ax,'Position (deg)');
title(pos6ax,'joint 6')
xlim(pos6ax,[T(1) T(n+1)]);

jntVelFig = figure('Name','Joint Vel Data Plot');
jntVelFig.Position = [450 120 1050 750];
vel1ax = subplot(3,2,1);
hold(vel1ax,'on')
plot(vel1ax,T(1):tstp:T(n+1),rad2deg(q_vel1(1,1:end)))
plot(vel1ax,T(1):tstp:T(n+1),rad2deg(q_vel2(1,1:end)))
plot(vel1ax,T(1):tstp:T(n+1),rad2deg(q_vel3(1,1:end)))
plot(vel1ax,T,rad2deg(VelC(1,:)),'g.')
plot(vel1ax,[T(1) T(n+1)],rad2deg(Vel(1,:)),'k.')
hold(vel1ax,'off')
grid(vel1ax,'on')
xlabel(vel1ax,'Time (sec)')
ylabel(vel1ax,'Velocity (deg/s)');
title(vel1ax,'joint 1')
xlim(vel1ax,[T(1) T(n+1)]);
vel2ax = subplot(3,2,2);
hold(vel2ax,'on')
plot(vel2ax,T(1):tstp:T(n+1),rad2deg(q_vel1(2,1:end)))
plot(vel2ax,T(1):tstp:T(n+1),rad2deg(q_vel2(2,1:end)))
plot(vel2ax,T(1):tstp:T(n+1),rad2deg(q_vel3(2,1:end)))
plot(vel2ax,T,rad2deg(VelC(2,:)),'g.')
plot(vel2ax,[T(1) T(n+1)],rad2deg(Vel(2,:)),'k.')
hold(vel2ax,'off')
grid(vel2ax,'on')
xlabel(vel2ax,'Time (sec)')
ylabel(vel2ax,'Velocity (deg/s)');
title(vel2ax,'joint 2')
xlim(vel2ax,[T(1) T(n+1)]);
vel3ax = subplot(3,2,3);
hold(vel3ax,'on')
plot(vel3ax,T(1):tstp:T(n+1),rad2deg(q_vel1(3,1:end)))
plot(vel3ax,T(1):tstp:T(n+1),rad2deg(q_vel2(3,1:end)))
plot(vel3ax,T(1):tstp:T(n+1),rad2deg(q_vel3(3,1:end)))
plot(vel3ax,T,rad2deg(VelC(3,:)),'g.')
plot(vel3ax,[T(1) T(n+1)],rad2deg(Vel(3,:)),'k.')
hold(vel3ax,'off')
grid(vel3ax,'on')
xlabel(vel3ax,'Time (sec)')
ylabel(vel3ax,'Velocity (deg/s)');
title(vel3ax,'joint 3')
xlim(vel3ax,[T(1) T(n+1)]);
vel4ax = subplot(3,2,4);
hold(vel4ax,'on')
plot(vel4ax,T(1):tstp:T(n+1),rad2deg(q_vel1(4,1:end)))
plot(vel4ax,T(1):tstp:T(n+1),rad2deg(q_vel2(4,1:end)))
plot(vel4ax,T(1):tstp:T(n+1),rad2deg(q_vel3(4,1:end)))
plot(vel4ax,T,rad2deg(VelC(4,:)),'g.')
plot(vel4ax,[T(1) T(n+1)],rad2deg(Vel(4,:)),'k.')
hold(vel4ax,'off')
grid(vel4ax,'on')
xlabel(vel4ax,'Time (sec)')
ylabel(vel4ax,'Velocity (deg/s)');
title(vel4ax,'joint 4')
xlim(vel4ax,[T(1) T(n+1)]);
vel5ax = subplot(3,2,5);
hold(vel5ax,'on')
plot(vel5ax,T(1):tstp:T(n+1),rad2deg(q_vel1(5,1:end)))
plot(vel5ax,T(1):tstp:T(n+1),rad2deg(q_vel2(5,1:end)))
plot(vel5ax,T(1):tstp:T(n+1),rad2deg(q_vel3(5,1:end)))
plot(vel5ax,T,rad2deg(VelC(5,:)),'g.')
plot(vel5ax,[T(1) T(n+1)],rad2deg(Vel(5,:)),'k.')
hold(vel5ax,'off')
grid(vel5ax,'on')
xlabel(vel5ax,'Time (sec)')
ylabel(vel5ax,'Velocity (deg/s)');
title(vel5ax,'joint 5')
xlim(vel5ax,[T(1) T(n+1)]);
vel6ax = subplot(3,2,6);
hold(vel6ax,'on')
plot(vel6ax,T(1):tstp:T(n+1),rad2deg(q_vel1(6,1:end)))
plot(vel6ax,T(1):tstp:T(n+1),rad2deg(q_vel2(6,1:end)))
plot(vel6ax,T(1):tstp:T(n+1),rad2deg(q_vel3(6,1:end)))
plot(vel6ax,T,rad2deg(VelC(6,:)),'g.')
plot(vel6ax,[T(1) T(n+1)],rad2deg(Vel(6,:)),'k.')
hold(vel6ax,'off')
grid(vel6ax,'on')
xlabel(vel6ax,'Time (sec)')
ylabel(vel6ax,'Velocity (deg/s)');
title(vel6ax,'joint 6')
xlim(vel6ax,[T(1) T(n+1)]);

% jnt1Fig = figure('Name','Joint 1 Data Plot');
% jnt1Fig.Position = [550 120 850 450];
% j1posax = subplot(4,1,1);
% plot(j1posax,T(1):tstp:T(n+1),rad2deg(q_pos1(1,:)))
% grid(j1posax,'on')
% xlabel(j1posax,'Time (sec)')
% ylabel(j1posax,'Position (deg)');
% j1velax = subplot(4,1,2);
% plot(j1velax,T(1):tstp:T(n+1),rad2deg(q_vel1(1,:)))
% grid(j1velax,'on')
% xlabel(j1velax,'Time (sec)')
% ylabel(j1velax,'Velocity (deg/s)');
% j1accax = subplot(4,1,3);
% plot(j1accax,T(1):tstp:T(n+1),rad2deg(q_acc1(1,:)))
% grid(j1accax,'on')
% xlabel(j1accax,'Time (sec)')
% ylabel(j1accax,'Acceleration (deg/s^2)');
% j1jerax = subplot(4,1,4);
% plot(j1jerax,T(1):tstp:T(n+1),rad2deg(q_jer1(1,:)))
% grid(j1jerax,'on')
% xlabel(j1jerax,'Time (sec)')
% ylabel(j1jerax,'Jerk (deg/s^3)');

% jntPosFig2 = figure('Name','Joint Pos Data Plot 2');
% jntPosFig2.Position = [975 120 850 750];
% pos1ax2 = subplot(3,2,1);
% hold(pos1ax2,'on')
% plot(pos1ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(1,1:end)))
% plot(pos1ax2,T,rad2deg(Pos(1,:)),'ro')
% hold(pos1ax2,'off')
% grid(pos1ax2,'on')
% xlabel(pos1ax2,'Time (sec)')
% ylabel(pos1ax2,'Position (deg)');
% title(pos1ax2,'joint 1')
% xlim(pos1ax2,[T(1) T(n+1)]);
% pos2ax2 = subplot(3,2,2);
% hold(pos2ax2,'on')
% plot(pos2ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(2,1:end)))
% plot(pos2ax2,T,rad2deg(Pos(2,:)),'ro')
% hold(pos2ax2,'off')
% grid(pos2ax2,'on')
% xlabel(pos2ax2,'Time (sec)')
% ylabel(pos2ax2,'Position (deg)');
% title(pos2ax2,'joint 2')
% xlim(pos2ax2,[T(1) T(n+1)]);
% pos3ax2 = subplot(3,2,3);
% hold(pos3ax2,'on')
% plot(pos3ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(3,1:end)))
% plot(pos3ax2,T,rad2deg(Pos(3,:)),'ro')
% hold(pos3ax2,'off')
% grid(pos3ax2,'on')
% xlabel(pos3ax2,'Time (sec)')
% ylabel(pos3ax2,'Position (deg)');
% title(pos3ax2,'joint 3')
% xlim(pos3ax2,[T(1) T(n+1)]);
% pos4ax2 = subplot(3,2,4);
% hold(pos4ax2,'on')
% plot(pos4ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(4,1:end)))
% plot(pos4ax2,T,rad2deg(Pos(4,:)),'ro')
% hold(pos4ax2,'off')
% grid(pos4ax2,'on')
% xlabel(pos4ax2,'Time (sec)')
% ylabel(pos4ax2,'Position (deg)');
% title(pos4ax2,'joint 4')
% xlim(pos4ax2,[T(1) T(n+1)]);
% pos5ax2 = subplot(3,2,5);
% hold(pos5ax2,'on')
% plot(pos5ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(5,1:end)))
% plot(pos5ax2,T,rad2deg(Pos(5,:)),'ro')
% hold(pos5ax2,'off')
% grid(pos5ax2,'on')
% xlabel(pos5ax2,'Time (sec)')
% ylabel(pos5ax2,'Position (deg)');
% title(pos5ax2,'joint 5')
% xlim(pos5ax2,[T(1) T(n+1)]);
% pos6ax2 = subplot(3,2,6);
% hold(pos6ax2,'on')
% plot(pos6ax2,T(1):tstp:T(n+1),rad2deg(q_pos2(6,1:end)))
% plot(pos6ax2,T,rad2deg(Pos(6,:)),'ro')
% hold(pos6ax2,'off')
% grid(pos6ax2,'on')
% xlabel(pos6ax2,'Time (sec)')
% ylabel(pos6ax2,'Position (deg)');
% title(pos6ax2,'joint 6')
% xlim(pos6ax2,[T(1) T(n+1)]);


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