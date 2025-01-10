clc, clear all, close all

trj = 2;

read_on = 0;
write_on = 0;
task_space = 1;

if write_on == 1
	tstp = 0.008;
else
    tstp = 0.1;
end

if trj == 1
T = [0 2 4 6 8 10]*2;

Pos = [ 0.4000	0.4000  0.5500  0.5500  0.4500  0.42     % x-axis
        0.2500	0.3500  0.1500  0.0500  0.1800  0.22     % y-axis
        0.9500  0.9100  0.9300  0.8500  0.8200  0.85];	% z-axis

euler_set = 1; % 1: ZYZ, 2: ZYX, 3: XYZ, 4:ZXZ
    
Orn = deg2rad([ 0	40	10	-30	0	10       % x-axis
                90	60	90	110	120	90      % y-axis
                0	0	20	-20	-30	-60]);	% z-axis
end
if trj == 2
    
    T = [0 4 6 10 14 16 20];

    Pos = [ 0.3900	0.4000  0.4000	0.4000	0.4000  0.4000  0.3900      % x-axis
            0.2500	0.3500  0.3500  0.3500  0.3500  0.3500  0.2500      % y-axis
            0.9500  0.9600  0.8800  0.8800  0.8800  0.9600  0.9500];	% z-axis
        
    % Pos = Pos - [0.1874;0;0]; % if no gripper
        
    euler_set = 1; % 1: ZYZ, 2: ZYX, 3: XYZ, 4:ZXZ

    Orn = deg2rad([ 0	0	0	60	0	0   0     % phi-axis
                    90.001	90	90	90	90	90  90     % theta-axis
                    0	90	90	30	90	90  0]);	% psi-axis

end
            
Vel = [ 0 0
        0 0
        0 0];
    
Acc = [ 0 0
        0 0
        0 0];

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
animFig = figure('Name','Animation Plot','WindowState','maximized');
animPlot = axes('Parent',animFig);

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

pCFP1 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP1, 'facec', 'flat');                      % Set the face color flat
set(pCFP1, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP1, 'EdgeColor', 'none');

pCFP2 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP2, 'facec', 'flat');                      % Set the face color flat
set(pCFP2, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP2, 'EdgeColor', 'none');

pCFP3 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP3, 'facec', 'flat');                      % Set the face color flat
set(pCFP3, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP3, 'EdgeColor', 'none');

pCFP4 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP4, 'facec', 'flat');                      % Set the face color flat
set(pCFP4, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP4, 'EdgeColor', 'none');

if trj == 1
pCFP5 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP5, 'facec', 'flat');                      % Set the face color flat
set(pCFP5, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP5, 'EdgeColor', 'none');

pCFP6 = patch(animPlot,'Faces', sCF.modelstruct.F, 'Vertices', sCF.modelstruct.V);
set(pCFP6, 'facec', 'flat');                      % Set the face color flat
set(pCFP6, 'FaceVertexCData', sCF.modelstruct.C);	% Set the color (from file)
set(pCFP6, 'EdgeColor', 'none');
end

[a_i,alpha_i,d_i,theta_i_O] = kinematicParametersUR3;

TI_0 = [rot_x(-pi/2)*rot_z(-pi/2),[0;-0.1;1];zeros(1,3),1];
% TI_0 = eye(4);
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

d_e = Tn_sC(3,4)+TsC_S(3,4)+TS_gC(3,4)+TgC_gB(3,4)+TgB_E(3,4);

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
        
kinPara.a_i = a_i;
kinPara.alpha_i = alpha_i;
kinPara.d_i = d_i;
kinPara.theta_i_O = theta_i_O;
kinPara.d_e = d_e;
kinPara.TI_0 = TI_0;
kinPara.q_posLim = q_posLim;
kinPara.inv_geo_trn = inv_geo_trn;
kinPara.kp_inv = kp_inv;
kinPara.kr_inv = kr_inv;
kinPara.kp_trn = kp_trn;
kinPara.kr_trn = kr_trn;
kinPara.wp = wp;
kinPara.wr = wr;
        
light(animPlot)     % add a default light
view(animPlot,[-225,30])	% Isometric view
grid(animPlot,'on')
xlabel(animPlot,'X'),ylabel(animPlot,'Y'),zlabel(animPlot,'Z')
set(animPlot,	'Projection','perspective', ...
                'PlotBoxAspectRatio',[1 1 1], ...
                'DataAspectRatio',[1 1 1]);

temp = [s0.modelstruct.V,ones(size(s0.modelstruct.V(:,1)))]*TI_0';
set(p0,'Vertices',temp(:,1:3))

n = length(T)-1;  % n= number of trajectories

TI_CFP1 = [getRotMatfromEA(Orn(:,1),euler_set),Pos(:,1);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP1';
set(pCFP1,'Vertices',temp(:,1:3),'FaceAlpha',0.25)

TI_CFP2 = [getRotMatfromEA(Orn(:,2),euler_set),Pos(:,2);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP2';
set(pCFP2,'Vertices',temp(:,1:3),'FaceAlpha',0.25)

TI_CFP3 = [getRotMatfromEA(Orn(:,3),euler_set),Pos(:,3);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP3';
set(pCFP3,'Vertices',temp(:,1:3),'FaceAlpha',0.25)

TI_CFP4 = [getRotMatfromEA(Orn(:,4),euler_set),Pos(:,4);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP4';
set(pCFP4,'Vertices',temp(:,1:3),'FaceAlpha',0.25)

if trj == 1
TI_CFP5 = [getRotMatfromEA(Orn(:,5),euler_set),Pos(:,5);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP5';
set(pCFP5,'Vertices',temp(:,1:3),'FaceAlpha',0.25)

TI_CFP6 = [getRotMatfromEA(Orn(:,6),euler_set),Pos(:,6);zeros(1,3),1];
temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_CFP6';
set(pCFP6,'Vertices',temp(:,1:3),'FaceAlpha',0.25)
end
% for i = 1:n+1
%     PosTemp = TI_0*[Pos(:,i);1];
%     Pos(:,i) = PosTemp(1:3);
% end
% Cj = zeros(6*n,6);
% for j = 1:6
%     Cj(:,j) = minimumJerkCoefficient(T,Pos(j,:),Vel(j,:),Acc(j,:));
% end

Cj = minimumJerkCoefficient(T,Pos,Vel,Acc);
Coj = minimumJerkCoefficient(T,Orn,Vel,Acc);

total_step = length(T(1):tstp:T(n+1));

q_pos = zeros(6,total_step);
% q_vel = zeros(6,total_step);
% q_acc = zeros(6,total_step);
% q_jer = zeros(6,total_step);

t_pos = zeros(3,total_step);
t_vel = zeros(3,total_step);
t_acc = zeros(3,total_step);
t_jer = zeros(3,total_step);
t_fbk = zeros(3,total_step);

r_pos = zeros(3,total_step);
r_vel = zeros(3,total_step);
r_acc = zeros(3,total_step);
r_jer = zeros(3,total_step);
r_fbk = zeros(3,total_step);

t_bas = zeros(3,total_step);

theta = zeros(1,total_step);
r_axs = zeros(3,total_step);
e_vec = zeros(3,total_step);

animFig.Position = [550 120 850 750];
% animFig.Position = [130 120 750 750];
% datXFig.Position = [880 120 320 750];
% datYFig.Position = [1200 120 320 750];
% datZFig.Position = [1520 120 320 750];

% initialization
RGoal = getRotMatfromEA(Orn(:,1),euler_set);

TGoal = [RGoal,Pos(:,1);zeros(1,3),1];
    
[q_pos(:,1),err] = invKinAnalytic(kinPara,TGoal,zeros(6,1),1);
if err == 1
    q_pos(:,1) = invGeoNumeric(kinPara,TGoal,zeros(6,1));
end
% q_pos(:,1)

t = T(1);
for k = 1:total_step
    [t_pos(:,k),t_vel(:,k),t_acc(:,k),t_jer(:,k)] = minimumJerkPolynomial(t,T,Cj);
    [r_pos(:,k),r_vel(:,k),r_acc(:,k),r_jer(:,k)] = minimumJerkPolynomial(t,T,Coj);
    t = round(t + tstp,3);
end

TI2_0 = [rot_x(-pi/2)*rot_z(+pi/2),[0;-0.1;1];zeros(1,3),1];

t = T(1);
qPrev = q_pos(:,1);
for k = 1:total_step
    
%     [t_pos(:,k),t_vel(:,k),t_acc(:,k),t_jer(:,k)] = minimumJerkPolynomial(t,T,Cj);
%     [r_pos(:,k),r_vel(:,k),r_acc(:,k),r_jer(:,k)] = minimumJerkPolynomial(t,T,Coj);
    
    RGoal = getRotMatfromEA(r_pos(:,k),euler_set);
    TGoal = [RGoal,t_pos(:,k);zeros(1,3),1];
    
    TBase = TI2_0\TGoal;
%     t_bas(:,k) = TBase(1:3,4);
%     
%     [theta(:,k),r_axs(:,k)] = getAxisAngle(TBase(1:3,1:3));
%     e_vec(:,k) = theta(:,k)*r_axs(:,k);
    
%     [q_pos(:,k),err] = invKinAnalytic(kinPara,TGoal,qPrev,0);
    [q_pos(:,k),err] = invKinAnaBasic(kinPara,TGoal,qPrev);
    if err == 1
        q_pos(:,k) = invGeoNumeric(kinPara,TGoal,qPrev);
    end

    [T0_h,~] = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+q_pos(:,k));

    t_bas(:,k) = T0_h(1:3,4,n+1);
    
    [theta(:,k),r_axs(:,k)] = getAxisAngle(T0_h(1:3,1:3,n+1));
    e_vec(:,k) = theta(:,k)*r_axs(:,k);
    
    for i = 1:6
        TI_i(:,:,i) = TI_0*T0_h(:,:,i+1);
    end

    TI_sC = TI_i(:,:,6)*Tn_sC;
    TI_S = TI_sC*TsC_S;
    TI_gC = TI_S*TS_gC;
    TI_gB = TI_gC*TgC_gB;
    TI_E = TI_gB*TgB_E;
    TI_oKa = TI_gB*TgB_oKa;
    TI_oKb = TI_gB*TgB_oKb;
    TI_oFa = TI_gB*TgB_oFa;
    TI_oFb = TI_gB*TgB_oFb;
    TI_iKa = TI_gB*TgB_iKa;
    TI_iKb = TI_gB*TgB_iKb;
    TI_iFa = TI_gB*TgB_iFa;
    TI_iFb = TI_gB*TgB_iFb;
    
    t_fbk(:,k) = TI_E(1:3,4);
    r_fbk(:,k) = getEulerPosVec(TI_E(1:3,1:3),euler_set);

    % transformation
    temp = [s1.modelstruct.V,ones(size(s1.modelstruct.V(:,1)))]*TI_i(:,:,1)';
    set(p1,'Vertices',temp(:,1:3))
    temp = [s2.modelstruct.V,ones(size(s2.modelstruct.V(:,1)))]*TI_i(:,:,2)';
    set(p2,'Vertices',temp(:,1:3))
    temp = [s3.modelstruct.V,ones(size(s3.modelstruct.V(:,1)))]*TI_i(:,:,3)';
    set(p3,'Vertices',temp(:,1:3))
    temp = [s4.modelstruct.V,ones(size(s4.modelstruct.V(:,1)))]*TI_i(:,:,4)';
    set(p4,'Vertices',temp(:,1:3))
    temp = [s5.modelstruct.V,ones(size(s5.modelstruct.V(:,1)))]*TI_i(:,:,5)';
    set(p5,'Vertices',temp(:,1:3))
    temp = [s6.modelstruct.V,ones(size(s6.modelstruct.V(:,1)))]*TI_i(:,:,6)';
    set(p6,'Vertices',temp(:,1:3))
    temp = [sSC.modelstruct.V,ones(size(sSC.modelstruct.V(:,1)))]*TI_sC';
    set(pSC,'Vertices',temp(:,1:3))
    temp = [sS.modelstruct.V,ones(size(sS.modelstruct.V(:,1)))]*TI_S';
    set(pS,'Vertices',temp(:,1:3))
    temp = [sGC.modelstruct.V,ones(size(sGC.modelstruct.V(:,1)))]*TI_gC';
    set(pGC,'Vertices',temp(:,1:3))
    temp = [sGB.modelstruct.V,ones(size(sGB.modelstruct.V(:,1)))]*TI_gB';
    set(pGB,'Vertices',temp(:,1:3))
    temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKa';
    set(pOKa,'Vertices',temp(:,1:3))
    temp = [sOK.modelstruct.V,ones(size(sOK.modelstruct.V(:,1)))]*TI_oKb';
    set(pOKb,'Vertices',temp(:,1:3))
    temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFa';
    set(pOFa,'Vertices',temp(:,1:3))
    temp = [sOF.modelstruct.V,ones(size(sOF.modelstruct.V(:,1)))]*TI_oFb';
    set(pOFb,'Vertices',temp(:,1:3))
    temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKa';
    set(pIKa,'Vertices',temp(:,1:3))
    temp = [sIK.modelstruct.V,ones(size(sIK.modelstruct.V(:,1)))]*TI_iKb';
    set(pIKb,'Vertices',temp(:,1:3))
    temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFa';
    set(pIFa,'Vertices',temp(:,1:3))
    temp = [sIF.modelstruct.V,ones(size(sIF.modelstruct.V(:,1)))]*TI_iFb';
    set(pIFb,'Vertices',temp(:,1:3))
    temp = [sCF.modelstruct.V,ones(size(sCF.modelstruct.V(:,1)))]*TI_E';
    set(pCF,'Vertices',temp(:,1:3))
    
    hold(animPlot,'on')
    grid(animPlot,'on')
    plot3(animPlot,t_pos(1,:),t_pos(2,:),t_pos(3,:),':b')
    plot3(animPlot,t_fbk(1,1:k),t_fbk(2,1:k),t_fbk(3,1:k),'b')
    plot3(animPlot,Pos(1,:),Pos(2,:),Pos(3,:),'ro')
    hold(animPlot,'off')
    if write_on == 0
        drawnow
    end
    
%     if rem(k,2) == 0 || k == total_step
%     end

    qPrev = q_pos(:,k);
    t = round(t + tstp,2);
end

datBasFig = figure('Name','Base Task Data Plot');
basA1ax = subplot(2,3,1);
title(basA1ax,'X')
plot(basA1ax,T(1):tstp:T(n+1),t_bas(1,:))
grid(basA1ax,'on')
xlabel(basA1ax,'Time (sec)')
ylabel(basA1ax,'Position (m)');
xlim(basA1ax,[T(1) T(n+1)])
basA2ax = subplot(2,3,2);
title(basA2ax,'Y')
plot(basA2ax,T(1):tstp:T(n+1),t_bas(2,:))
grid(basA2ax,'on')
xlabel(basA2ax,'Time (sec)')
ylabel(basA2ax,'Position (m)');
xlim(basA2ax,[T(1) T(n+1)])
basA3ax = subplot(2,3,3);
title(basA3ax,'Z')
plot(basA3ax,T(1):tstp:T(n+1),t_bas(3,:))
grid(basA3ax,'on')
xlabel(basA3ax,'Time (sec)')
ylabel(basA3ax,'Position (m)');
xlim(basA3ax,[T(1) T(n+1)])
basB1ax = subplot(2,3,4);
title(basB1ax,'Rx')
plot(basB1ax,T(1):tstp:T(n+1),e_vec(1,:))
grid(basB1ax,'on')
xlabel(basB1ax,'Time (sec)')
ylabel(basB1ax,'Orientation (rad)');
xlim(basB1ax,[T(1) T(n+1)])
basB2ax = subplot(2,3,5);
title(basB2ax,'Ry')
plot(basB2ax,T(1):tstp:T(n+1),e_vec(2,:))
grid(basB2ax,'on')
xlabel(basB2ax,'Time (sec)')
ylabel(basB2ax,'Orientation (rad)');
xlim(basB2ax,[T(1) T(n+1)])
basB3ax = subplot(2,3,6);
title(basB3ax,'Rz')
plot(basB3ax,T(1):tstp:T(n+1),e_vec(3,:))
grid(basB3ax,'on')
xlabel(basB3ax,'Time (sec)')
ylabel(basB3ax,'Orientation (rad)');
xlim(basB3ax,[T(1) T(n+1)])
% axsB1ax = subplot(4,3,4);
% hold(axsB1ax,'on')
% plot(axsB1ax,T(1):tstp:T(n+1),rad2deg(r_vel(1,1:end)))
% plot(axsB1ax,[T(1) T(n+1)],[Vel(1,1) Vel(1,2)],'ro')
% hold(axsB1ax,'off')
% grid(axsB1ax,'on')
% xlabel(axsB1ax,'Time (sec)')
% ylabel(axsB1ax,'Velocity (deg/sec)');
% xlim(axsB1ax,[T(1) T(n+1)])
% axsB2ax = subplot(4,3,5);
% hold(axsB2ax,'on')
% plot(axsB2ax,T(1):tstp:T(n+1),rad2deg(r_vel(2,1:end)))
% plot(axsB2ax,[T(1) T(n+1)],[Vel(2,1) Vel(2,2)],'ro')
% hold(axsB2ax,'off')
% grid(axsB2ax,'on')
% xlabel(axsB2ax,'Time (sec)')
% ylabel(axsB2ax,'Velocity (deg/sec)');
% xlim(axsB2ax,[T(1) T(n+1)])
% axsB3ax = subplot(4,3,6);
% hold(axsB3ax,'on')
% plot(axsB3ax,T(1):tstp:T(n+1),rad2deg(r_vel(3,1:end)))
% plot(axsB3ax,[T(1) T(n+1)],[Vel(3,1) Vel(3,2)],'ro')
% hold(axsB3ax,'off')
% grid(axsB3ax,'on')
% xlabel(axsB3ax,'Time (sec)')
% ylabel(axsB3ax,'Velocity (deg/sec)');
% xlim(axsB3ax,[T(1) T(n+1)])
% ornC1ax = subplot(4,3,7);
% hold(ornC1ax,'on')
% plot(ornC1ax,T(1):tstp:T(n+1),rad2deg(r_acc(1,1:end)))
% plot(ornC1ax,[T(1) T(n+1)],[Acc(1,1) Acc(1,2)],'ro')
% hold(ornC1ax,'off')
% grid(ornC1ax,'on')
% xlabel(ornC1ax,'Time (sec)')
% ylabel(ornC1ax,'Acceleration (deg/sec^2)');
% xlim(ornC1ax,[T(1) T(n+1)])
% ornC2ax = subplot(4,3,8);
% hold(ornC2ax,'on')
% plot(ornC2ax,T(1):tstp:T(n+1),rad2deg(r_acc(2,1:end)))
% plot(ornC2ax,[T(1) T(n+1)],[Acc(2,1) Acc(2,2)],'ro')
% hold(ornC2ax,'off')
% grid(ornC2ax,'on')
% xlabel(ornC2ax,'Time (sec)')
% ylabel(ornC2ax,'Acceleration (deg/sec^2)');
% xlim(ornC2ax,[T(1) T(n+1)])
% ornC3ax = subplot(4,3,9);
% hold(ornC3ax,'on')
% plot(ornC3ax,T(1):tstp:T(n+1),rad2deg(r_acc(3,1:end)))
% plot(ornC3ax,[T(1) T(n+1)],[Acc(3,1) Acc(3,2)],'ro')
% hold(ornC3ax,'off')
% grid(ornC3ax,'on')
% xlabel(ornC3ax,'Time (sec)')
% ylabel(ornC3ax,'Acceleration (deg/sec^2)');
% xlim(ornC3ax,[T(1) T(n+1)])
% ornD1ax = subplot(4,3,10);
% plot(ornD1ax,T(1):tstp:T(n+1),rad2deg(r_jer(1,1:end)))
% grid(ornD1ax,'on')
% xlabel(ornD1ax,'Time (sec)')
% ylabel(ornD1ax,'Jerk (deg/sec^3)');
% xlim(ornD1ax,[T(1) T(n+1)])
% ornD2ax = subplot(4,3,11);
% plot(ornD2ax,T(1):tstp:T(n+1),rad2deg(r_jer(2,1:end)))
% grid(ornD2ax,'on')
% xlabel(ornD2ax,'Time (sec)')
% ylabel(ornD2ax,'Jerk (deg/sec^3)');
% xlim(ornD2ax,[T(1) T(n+1)])
% ornD3ax = subplot(4,3,12);
% plot(ornD3ax,T(1):tstp:T(n+1),rad2deg(r_jer(3,1:end)))
% grid(ornD3ax,'on')
% xlabel(ornD3ax,'Time (sec)')
% ylabel(ornD3ax,'Jerk (deg/sec^3)');
% xlim(ornD3ax,[T(1) T(n+1)])

datOrnFig = figure('Name','Euler Angles Data Plot');
ornA1ax = subplot(4,3,1);
title(ornA1ax,'Z(I)')
hold(ornA1ax,'on')
plot(ornA1ax,T(1):tstp:T(n+1),rad2deg([r_pos(1,1:end);r_fbk(1,1:end)]))
plot(ornA1ax,T',rad2deg(Orn(1,:)),'ro')
hold(ornA1ax,'off')
grid(ornA1ax,'on')
xlabel(ornA1ax,'Time (sec)')
ylabel(ornA1ax,'Position (deg)');
xlim(ornA1ax,[T(1) T(n+1)])
ornA2ax = subplot(4,3,2);
title(ornA2ax,'Y')
hold(ornA2ax,'on')
plot(ornA2ax,T(1):tstp:T(n+1),rad2deg([r_pos(2,1:end);r_fbk(2,1:end)]))
plot(ornA2ax,T',rad2deg(Orn(2,:)),'ro')
hold(ornA2ax,'off')
grid(ornA2ax,'on')
xlabel(ornA2ax,'Time (sec)')
ylabel(ornA2ax,'Position (deg)');
xlim(ornA2ax,[T(1) T(n+1)])
ornA3ax = subplot(4,3,3);
title(ornA3ax,'Z(II)')
hold(ornA3ax,'on')
plot(ornA3ax,T(1):tstp:T(n+1),rad2deg([r_pos(3,1:end);r_fbk(3,1:end)]))
plot(ornA3ax,T',rad2deg(Orn(3,:)),'ro')
hold(ornA3ax,'off')
grid(ornA3ax,'on')
xlabel(ornA3ax,'Time (sec)')
ylabel(ornA3ax,'Position (deg)');
xlim(ornA3ax,[T(1) T(n+1)])
ornB1ax = subplot(4,3,4);
hold(ornB1ax,'on')
plot(ornB1ax,T(1):tstp:T(n+1),rad2deg(r_vel(1,1:end)))
plot(ornB1ax,[T(1) T(n+1)],[Vel(1,1) Vel(1,2)],'ro')
hold(ornB1ax,'off')
grid(ornB1ax,'on')
xlabel(ornB1ax,'Time (sec)')
ylabel(ornB1ax,'Velocity (deg/sec)');
xlim(ornB1ax,[T(1) T(n+1)])
ornB2ax = subplot(4,3,5);
hold(ornB2ax,'on')
plot(ornB2ax,T(1):tstp:T(n+1),rad2deg(r_vel(2,1:end)))
plot(ornB2ax,[T(1) T(n+1)],[Vel(2,1) Vel(2,2)],'ro')
hold(ornB2ax,'off')
grid(ornB2ax,'on')
xlabel(ornB2ax,'Time (sec)')
ylabel(ornB2ax,'Velocity (deg/sec)');
xlim(ornB2ax,[T(1) T(n+1)])
ornB3ax = subplot(4,3,6);
hold(ornB3ax,'on')
plot(ornB3ax,T(1):tstp:T(n+1),rad2deg(r_vel(3,1:end)))
plot(ornB3ax,[T(1) T(n+1)],[Vel(3,1) Vel(3,2)],'ro')
hold(ornB3ax,'off')
grid(ornB3ax,'on')
xlabel(ornB3ax,'Time (sec)')
ylabel(ornB3ax,'Velocity (deg/sec)');
xlim(ornB3ax,[T(1) T(n+1)])
ornC1ax = subplot(4,3,7);
hold(ornC1ax,'on')
plot(ornC1ax,T(1):tstp:T(n+1),rad2deg(r_acc(1,1:end)))
plot(ornC1ax,[T(1) T(n+1)],[Acc(1,1) Acc(1,2)],'ro')
hold(ornC1ax,'off')
grid(ornC1ax,'on')
xlabel(ornC1ax,'Time (sec)')
ylabel(ornC1ax,'Acceleration (deg/sec^2)');
xlim(ornC1ax,[T(1) T(n+1)])
ornC2ax = subplot(4,3,8);
hold(ornC2ax,'on')
plot(ornC2ax,T(1):tstp:T(n+1),rad2deg(r_acc(2,1:end)))
plot(ornC2ax,[T(1) T(n+1)],[Acc(2,1) Acc(2,2)],'ro')
hold(ornC2ax,'off')
grid(ornC2ax,'on')
xlabel(ornC2ax,'Time (sec)')
ylabel(ornC2ax,'Acceleration (deg/sec^2)');
xlim(ornC2ax,[T(1) T(n+1)])
ornC3ax = subplot(4,3,9);
hold(ornC3ax,'on')
plot(ornC3ax,T(1):tstp:T(n+1),rad2deg(r_acc(3,1:end)))
plot(ornC3ax,[T(1) T(n+1)],[Acc(3,1) Acc(3,2)],'ro')
hold(ornC3ax,'off')
grid(ornC3ax,'on')
xlabel(ornC3ax,'Time (sec)')
ylabel(ornC3ax,'Acceleration (deg/sec^2)');
xlim(ornC3ax,[T(1) T(n+1)])
ornD1ax = subplot(4,3,10);
plot(ornD1ax,T(1):tstp:T(n+1),rad2deg(r_jer(1,1:end)))
grid(ornD1ax,'on')
xlabel(ornD1ax,'Time (sec)')
ylabel(ornD1ax,'Jerk (deg/sec^3)');
xlim(ornD1ax,[T(1) T(n+1)])
ornD2ax = subplot(4,3,11);
plot(ornD2ax,T(1):tstp:T(n+1),rad2deg(r_jer(2,1:end)))
grid(ornD2ax,'on')
xlabel(ornD2ax,'Time (sec)')
ylabel(ornD2ax,'Jerk (deg/sec^3)');
xlim(ornD2ax,[T(1) T(n+1)])
ornD3ax = subplot(4,3,12);
plot(ornD3ax,T(1):tstp:T(n+1),rad2deg(r_jer(3,1:end)))
grid(ornD3ax,'on')
xlabel(ornD3ax,'Time (sec)')
ylabel(ornD3ax,'Jerk (deg/sec^3)');
xlim(ornD3ax,[T(1) T(n+1)])

datPosFig = figure('Name','Cartesian Coord Data Plot');
posxax = subplot(4,3,1);
title(posxax,'x-axis')
hold(posxax,'on')
plot(posxax,T(1):tstp:T(n+1),[t_pos(1,1:end);t_fbk(1,1:end)])
plot(posxax,T',Pos(1,:),'ro')
hold(posxax,'off')
grid(posxax,'on')
xlabel(posxax,'Time (sec)')
ylabel(posxax,'Position (m)');
xlim(posxax,[T(1) T(n+1)]);
posyax = subplot(4,3,2);
title(posyax,'y-axis')
hold(posyax,'on')
plot(posyax,T(1):tstp:T(n+1),[t_pos(2,1:end);t_fbk(2,1:end)])
plot(posyax,T',Pos(2,:),'ro')
hold(posyax,'off')
grid(posyax,'on')
xlabel(posyax,'Time (sec)')
ylabel(posyax,'Position (m)');
xlim(posyax,[T(1) T(n+1)]);
poszax = subplot(4,3,3);
title(poszax,'z-axis')
hold(poszax,'on')
plot(poszax,T(1):tstp:T(n+1),[t_pos(3,1:end);t_fbk(3,1:end)])
plot(poszax,T',Pos(3,:),'ro')
hold(poszax,'off')
grid(poszax,'on')
xlabel(poszax,'Time (sec)')
ylabel(poszax,'Position (m)');
xlim(poszax,[T(1) T(n+1)]);
velxax = subplot(4,3,4);
hold(velxax,'on')
plot(velxax,T(1):tstp:T(n+1),t_vel(1,1:end))
plot(velxax,[T(1) T(n+1)],[Vel(1,1) Vel(1,2)],'ro')
hold(velxax,'off')
grid(velxax,'on')
xlabel(velxax,'Time (sec)')
ylabel(velxax,'Velocity (m/sec)');
xlim(velxax,[T(1) T(n+1)])
velyax = subplot(4,3,5);
hold(velyax,'on')
plot(velyax,T(1):tstp:T(n+1),t_vel(2,1:end))
plot(velyax,[T(1) T(n+1)],[Vel(2,1) Vel(2,2)],'ro')
hold(velyax,'off')
grid(velyax,'on')
xlabel(velyax,'Time (sec)')
ylabel(velyax,'Velocity (m/sec)');
xlim(velyax,[T(1) T(n+1)])
velzax = subplot(4,3,6);
hold(velzax,'on')
plot(velzax,T(1):tstp:T(n+1),t_vel(3,1:end))
plot(velzax,[T(1) T(n+1)],[Vel(3,1) Vel(3,2)],'ro')
hold(velzax,'off')
grid(velzax,'on')
xlabel(velzax,'Time (sec)')
ylabel(velzax,'Velocity (m/sec)');
xlim(velzax,[T(1) T(n+1)])
accxax = subplot(4,3,7);
hold(accxax,'on')
plot(accxax,T(1):tstp:T(n+1),t_acc(1,1:end))
plot(accxax,[T(1) T(n+1)],[Vel(1,1) Vel(1,2)],'ro')
hold(accxax,'off')
grid(accxax,'on')
xlabel(accxax,'Time (sec)')
ylabel(accxax,'Acceleration (m/sec^2)');
xlim(accxax,[T(1) T(n+1)])
accyax = subplot(4,3,8);
hold(accyax,'on')
plot(accyax,T(1):tstp:T(n+1),t_acc(2,1:end))
plot(accyax,[T(1) T(n+1)],[Vel(2,1) Vel(2,2)],'ro')
hold(accyax,'off')
grid(accyax,'on')
xlabel(accyax,'Time (sec)')
ylabel(accyax,'Acceleration (m/sec^2)');
xlim(accyax,[T(1) T(n+1)])
acczax = subplot(4,3,9);
hold(acczax,'on')
plot(acczax,T(1):tstp:T(n+1),t_acc(3,1:end))
plot(acczax,[T(1) T(n+1)],[Vel(3,1) Vel(3,2)],'ro')
hold(acczax,'off')
grid(acczax,'on')
xlabel(acczax,'Time (sec)')
ylabel(acczax,'Acceleration (m/sec^2)');
xlim(acczax,[T(1) T(n+1)])
jerxax = subplot(4,3,10);
plot(jerxax,T(1):tstp:T(n+1),t_jer(1,:))
grid(jerxax,'on')
xlabel(jerxax,'Time (sec)')
ylabel(jerxax,'Jerk (m/sec^3)');
xlim(jerxax,[T(1) T(n+1)])
jeryax = subplot(4,3,11);
plot(jeryax,T(1):tstp:T(n+1),t_jer(2,:))
grid(jeryax,'on')
xlabel(jeryax,'Time (sec)')
ylabel(jeryax,'Jerk (m/sec^3)');
xlim(jeryax,[T(1) T(n+1)])
jerzax = subplot(4,3,12);
plot(jerzax,T(1):tstp:T(n+1),t_jer(3,:))
grid(jerzax,'on')
xlabel(jerzax,'Time (sec)')
ylabel(jerzax,'Jerk (m/sec^3)');
xlim(jerzax,[T(1) T(n+1)])

datOrnFig.Position = [110 120 850 750];
datPosFig.Position = [960 120 850 750];

if read_on == 1
    actual_q = readmatrix('log.csv');

    q_pos_read = actual_q';
    q_pos_read(2,:) = q_pos_read(2,:)+pi/2;
    q_pos_read(4,:) = q_pos_read(4,:)+pi/2;

    record_len = length(q_pos_read(1,:));
end

jntPosFig = figure('Name','Joint Pos Data Plot');
jntPosFig.Position = [550 120 850 450];
pos1ax = subplot(3,2,1);
plot(pos1ax,T(1):tstp:T(n+1),rad2deg(q_pos(1,1:end)))
if read_on == 1
    hold(pos1ax,'on')
    plot(pos1ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(1,:)))
    hold(pos1ax,'off')
end
grid(pos1ax,'on')
xlabel(pos1ax,'Time (sec)')
ylabel(pos1ax,'Position (deg)');
title(pos1ax,'joint 1')
xlim(pos1ax,[T(1) T(n+1)]);
pos2ax = subplot(3,2,2);
plot(pos2ax,T(1):tstp:T(n+1),rad2deg(q_pos(2,1:end)))
if read_on == 1
    hold(pos2ax,'on')
    plot(pos2ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(2,:)))
    hold(pos2ax,'off')
end
grid(pos2ax,'on')
xlabel(pos2ax,'Time (sec)')
ylabel(pos2ax,'Position (deg)');
title(pos2ax,'joint 2')
xlim(pos2ax,[T(1) T(n+1)]);
pos3ax = subplot(3,2,3);
plot(pos3ax,T(1):tstp:T(n+1),rad2deg(q_pos(3,1:end)))
if read_on == 1
    hold(pos3ax,'on')
    plot(pos3ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(3,:)))
    hold(pos3ax,'off')
end
grid(pos3ax,'on')
xlabel(pos3ax,'Time (sec)')
ylabel(pos3ax,'Position (deg)');
title(pos3ax,'joint 3')
xlim(pos3ax,[T(1) T(n+1)]);
pos4ax = subplot(3,2,4);
plot(pos4ax,T(1):tstp:T(n+1),rad2deg(q_pos(4,1:end)))
if read_on == 1
    hold(pos4ax,'on')
    plot(pos4ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(4,:)))
    hold(pos4ax,'off')
end
grid(pos4ax,'on')
xlabel(pos4ax,'Time (sec)')
ylabel(pos4ax,'Position (deg)');
title(pos4ax,'joint 4')
xlim(pos4ax,[T(1) T(n+1)]);
pos5ax = subplot(3,2,5);
plot(pos5ax,T(1):tstp:T(n+1),rad2deg(q_pos(5,1:end)))
if read_on == 1
    hold(pos5ax,'on')
    plot(pos5ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(5,:)))
    hold(pos5ax,'off')
end
grid(pos5ax,'on')
xlabel(pos5ax,'Time (sec)')
ylabel(pos5ax,'Position (deg)');
title(pos5ax,'joint 5')
xlim(pos5ax,[T(1) T(n+1)]);
pos6ax = subplot(3,2,6);
plot(pos6ax,T(1):tstp:T(n+1),rad2deg(q_pos(6,1:end)))
if read_on == 1
    hold(pos6ax,'on')
    plot(pos6ax,0:tstp:(record_len-1)*tstp,rad2deg(q_pos_read(6,:)))
    hold(pos6ax,'off')
end
grid(pos6ax,'on')
xlabel(pos6ax,'Time (sec)')
ylabel(pos6ax,'Position (deg)');
title(pos6ax,'joint 6')
xlim(pos6ax,[T(1) T(n+1)]);

if write_on == 1
    if task_space == 1
        x_pos_write = zeros(size(t_pos,2),6);
        x_pos_write(:,1:3) = t_bas';
        x_pos_write(:,4:6) = e_vec';
        
        writematrix(x_pos_write,'x_pos.csv')
    else
        q_pos_write = q_pos';
        q_pos_write(:,2) = q_pos_write(:,2)-pi/2;
        q_pos_write(:,4) = q_pos_write(:,4)-pi/2;

        writematrix(q_pos_write,'q_pos.csv')
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

function output = Trn_z(input)
    output = [  1 0 0 0
                0 1 0 0
                0 0 1 input
                0 0 0 1     ];
end

function [a_i,alpha_i,d_i,thetaPlus_i] = kinematicParametersUR3

    pO2 = pi/2;

    A1 = 0; A2 = -pO2; A3 = 0; A4 = 0; A5 = pO2; A6 = -pO2;
    T1 = 0; T2 = -pO2; T3 = 0; T4 = pO2; T5 = 0; T6 = 0;
    
    a1 = 0; a2 = 0; a3 = 0.24365; a4 = 0.21325; a5 = 0; a6 = 0;
    d1 = 0.1519; d2 = 0; d3 = 0; d4 = 0.11235; d5 = 0.08535; d6 = 0.0819;
    
    % General DH
    a_i     = [a1;a2;a3;a4;a5;a6];
    alpha_i = [A1;A2;A3;A4;A5;A6];
    d_i     = [d1;d2;d3;d4;d5;d6];
    thetaPlus_i = [T1;T2;T3;T4;T5;T6];
    
end

function Cj = minimumJerkCoefficient(T,Pos,Vel,Acc)

    [d,p] = size(Pos);  % d = # of dimensions, p = # of waypoints
    I = eye(d);
    Z = zeros(d);
    n = p-1;  % n= number of trajectories 

    % Waypoints' Matrix
    R = zeros(6*n*d,6*n*d); % Start with zero matrix for R, joint
    % Positions of all waypoints
    for i = 1:n
        R(1+d*(2*i-2):d*(2*i-1),1+6*d*(i-1):6*d*i)  = [I*T(i)^5      I*T(i)^4    I*T(i)^3    I*T(i)^2    I*T(i)   I];
        R(1+d*(2*i-1):d*(2*i+0),1+6*d*(i-1):6*d*i)	= [I*T(i+1)^5    I*T(i+1)^4  I*T(i+1)^3  I*T(i+1)^2  I*T(i+1) I];
    end
    % Velocity boundary conditions (inital and final waypoints)
    R(1+d*(0+2*n):d*(1+2*n),1:6*d            ) = [5*I*T(1)^4 	4*I*T(1)^3      3*I*T(1)^2      2*I*T(1)	I Z];
    R(1+d*(1+2*n):d*(2+2*n),1+6*d*(n-1):6*d*n) = [5*I*T(n+1)^4	4*I*T(n+1)^3	3*I*T(n+1)^2    2*I*T(n+1)  I Z];
    % Equal Accelaration boundary conditions (initial and final waypoints)
    R(1+d*(2+2*n):d*(3+2*n),1:6*d            ) = [20*I*T(1)^3	12*I*T(1)^2     6*I*T(1)	2*I	Z Z];
    R(1+d*(3+2*n):d*(4+2*n),1+6*d*(n-1):6*d*n) = [20*I*T(n+1)^3	12*I*T(n+1)^2	6*I*T(n+1)	2*I	Z Z];
    %Equal velocity, accelaration , jerk, and snap at intermideate waypoints
    for i = 1:n-1
        R(1+d*(4+(i-1)+0*(n-1)+2*n):d*(5+(i-1)+0*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [5*I*T(i+1)^4	4*I*T(i+1)^3	3*I*T(i+1)^2	2*I*T(i+1)	I Z	-5*I*T(i+1)^4	-4*I*T(i+1)^3	-3*I*T(i+1)^2	-2*I*T(i+1)	-I  Z]; % Equal velocity at intermediate waypoints
        R(1+d*(4+(i-1)+1*(n-1)+2*n):d*(5+(i-1)+1*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [20*I*T(i+1)^3	12*I*T(i+1)^2	6*I*T(i+1)      2*I        	Z Z	-20*I*T(i+1)^3	-12*I*T(i+1)^2	-6*I*T(i+1)     -2*I      	Z   Z]; % Equal acceleration at intermediate waypoints
        R(1+d*(4+(i-1)+2*(n-1)+2*n):d*(5+(i-1)+2*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [60*I*T(i+1)^2	24*I*T(i+1)     6*I           	Z           Z Z	-60*I*T(i+1)^2	-24*I*T(i+1)	-6*I            Z           Z   Z]; % Equal jerk at intermediate waypoints
        R(1+d*(4+(i-1)+3*(n-1)+2*n):d*(5+(i-1)+3*(n-1)+2*n),1+6*d*(i-1):6*(i+1)*d) = [120*I*T(i+1)	24*I        	Z               Z           Z Z -120*I*T(i+1)	-24*I         	Z               Z           Z   Z]; % Equal snap at intermediate waypoints
    end

    % Boundary Conditions Matrix
    BC = zeros(6*d*n,1);
    BC(1:d) = Pos(:,1);             % Position of the first waypoint
    if n > 1
        PosInter = zeros(2*(n-1),d);
    end
    for i = 2:n
        PosInter(2*(i-2)+1:2*(i-2)+2,:) = [Pos(:,i)';Pos(:,i)'];
    end
    for i = 1:2*(n-1)
        BC(1+d*i:d*(i+1)) = PosInter(i,:);
    end
    BC(1+d*(1+2*(n-1)):d*(2+2*(n-1))) = Pos(:,n+1); % Position of the final waypoint
    BC(1+d*(2+2*(n-1)):d*(3+2*(n-1))) = Vel(:,1);   % initial velocity
    BC(1+d*(3+2*(n-1)):d*(4+2*(n-1))) = Vel(:,2);   % final velocity
    BC(1+d*(4+2*(n-1)):d*(5+2*(n-1))) = Acc(:,1);   % initial acceleration
    BC(1+d*(5+2*(n-1)):d*(6+2*(n-1))) = Acc(:,2);   % final acceleration

    % Coefficient  Vector
    Cj = R\BC;
    
end

function [Pt,Vt,At,Jt] = minimumJerkPolynomial(t,T,Cj)

    n = length(T)-1;  % n = number of trajectories
    d = length(Cj)/(6*n);
    
    % Trajectory Polynomial
    for j = n+1:-1:2
        if t <= T(j)
            i = j-1;
        end
    end
    
    Pt = Cj(1+d*(6*i-6):d*(6*i-5))*t^5 + Cj(1+d*(6*i-5):d*(6*i-4))*t^4 + Cj(1+d*(6*i-4):d*(6*i-3))*t^3 + Cj(1+d*(6*i-3):d*(6*i-2))*t^2 + Cj(1+d*(6*i-2):d*(6*i-1))*t + Cj(1+d*(6*i-1):d*(6*i));
    Vt = 5*Cj(1+d*(6*i-6):d*(6*i-5))*t^4 + 4*Cj(1+d*(6*i-5):d*(6*i-4))*t^3 + 3*Cj(1+d*(6*i-4):d*(6*i-3))*t^2 + 2*Cj(1+d*(6*i-3):d*(6*i-2))*t + Cj(1+d*(6*i-2):d*(6*i-1));
    At = 5*4*Cj(1+d*(6*i-6):d*(6*i-5))*t^3 + 4*3*Cj(1+d*(6*i-5):d*(6*i-4))*t^2 + 3*2*Cj(1+d*(6*i-4):d*(6*i-3))*t + 2*Cj(1+d*(6*i-3):d*(6*i-2));
    Jt = 5*4*3*Cj(1+d*(6*i-6):d*(6*i-5))*t^2 + 4*3*2*Cj(1+d*(6*i-5):d*(6*i-4))*t + 3*2*Cj(1+d*(6*i-4):d*(6*i-3));

end

function qDes = invGeoNumeric(kinPara,TGoal,q0)

    a_i         = kinPara.a_i;
    alpha_i     = kinPara.alpha_i;
    d_i         = kinPara.d_i;
    theta_i_O   = kinPara.theta_i_O;
    d_e         = kinPara.d_e;
    TI_0        = kinPara.TI_0;
    q_posLim    = kinPara.q_posLim;
    wp = kinPara.wp;
    wr = kinPara.wr;

%     TGoalBase = TI_0'*TGoal;
%     pGoal = TGoalBase(1:3,4);
    pGoal = TGoal(1:3,4);

%     errNormPrev = 2;
    errNorm = 1;
    itr = 0;
    tol = 1e-5;

    if kinPara.inv_geo_trn == 0
        kp = kinPara.kp_inv;
        kr = kinPara.kr_inv;
    else
        kp = kinPara.kp_trn;
        kr = kinPara.kr_trn;
    end
    k = [kp;kp;kp;kr;kr;kr];

    J = zeros(6,6);

    qItr = q0;
%     while abs(errNorm-errNormPrev) > tol
    while abs(errNorm) > tol

        if itr > 5000
            break;
        end

        TItr = forwardGeometry(a_i,alpha_i,d_i,theta_i_O+qItr);
        TItr(:,:,6+1) = TItr(:,:,6+1)*Trn_z(d_e);
        for i = 1:7
            TItr(:,:,i) = TI_0*TItr(:,:,i);
        end

        for i = 1:6
            Jv = cross(TItr(1:3,3,i+1),(TItr(1:3,4,6+1)-TItr(1:3,4,i+1)));
            Jw = TItr(1:3,3,i+1);
            J(:,i) = [Jv;Jw];
        end

        pItr = TItr(1:3,4,6+1);
        pErr = pGoal-pItr;
        rErr = 0.5*(cross(TItr(1:3,1,6+1),TGoal(1:3,1))+cross(TItr(1:3,2,6+1),TGoal(1:3,2))+cross(TItr(1:3,3,6+1),TGoal(1:3,3)));
%         xErr = [pErr;rErr];
        xErr = [wp*pErr;wr*rErr];
%         errNormPrev = errNorm;
        errNorm = norm(xErr);

        if kinPara.inv_geo_trn == 0
            qItr = qItr + k.*(J'*xErr);
        else
            qItr = qItr + k.*(J\xErr);
        end

        itr = itr + 1;
    end
    
    for i = 1:6
        if qItr(i) < deg2rad(q_posLim(i,1))
            qItr(i) = deg2rad(q_posLim(i,1));
        elseif qItr(i) > deg2rad(q_posLim(i,2))
            qItr(i) = deg2rad(q_posLim(i,2));
        end
    end

    qDes = qItr;

end

function R_mat = getRotMatfromEA(EA_vec,EA_set)
    
	CPHI = cos(EA_vec(1));
	CTHT = cos(EA_vec(2));
	CPSI = cos(EA_vec(3));

	SPHI = sin(EA_vec(1));
	STHT = sin(EA_vec(2));
	SPSI = sin(EA_vec(3));

    switch EA_set
        case 1 % ZYZ
            R_mat = [   CTHT*CPHI*CPSI-SPHI*SPSI, -CPSI*SPHI-CTHT*CPHI*SPSI , CPHI*STHT
                        CPHI*SPSI+CTHT*CPSI*SPHI, CPHI*CPSI-CTHT*SPHI*SPSI  , STHT*SPHI
                        -CPSI*STHT              , STHT*SPSI                 , CTHT      ];
        case 2 % ZYX
            R_mat = [   CTHT*CPHI   , CPHI*SPSI*STHT-CPSI*SPHI  , SPSI*SPHI+CPSI*CPHI*STHT
                        CTHT*SPHI   , CPSI*CPHI+SPSI*STHT*SPHI  , CPSI*STHT*SPHI-CPHI*SPSI
                        -STHT   	, CTHT*SPSI                 , CPSI*CTHT             ];
        case 3 % XYZ
            R_mat = [   CTHT*CPSI               , -CTHT*SPSI                , STHT
                        CPHI*SPSI+CPSI*SPHI*STHT, CPHI*CPSI-SPHI*STHT*SPSI  , -CTHT*SPHI
                        SPHI*SPSI-CPHI*CPSI*STHT, CPSI*SPHI+CPHI*STHT*SPSI  , CPHI*CTHT ];
        otherwise % ZXZ
            R_mat = [   CPHI*CPSI-CTHT*SPHI*SPSI, -CPHI*SPSI-CTHT*CPSI*SPHI , STHT*SPHI
                        CPSI*SPHI+CTHT*CPHI*SPSI, CTHT*CPHI*CPSI-SPHI*SPSI  , -CPHI*STHT
                        STHT*SPSI               , CPSI*STHT                 , CTHT      ];
    end
    
end

function r_pos = getEulerPosVec(R_mat, eulerSet)

    sx = R_mat(1,1);
    sy = R_mat(2,1);
    sz = R_mat(3,1);
    nx = R_mat(1,2);
    ny = R_mat(2,2);
    nz = R_mat(3,2);
    ax = R_mat(1,3);
    ay = R_mat(2,3);
    az = R_mat(3,3);

    switch eulerSet
        case 1 % ZYZ
            phi = atan2(ay, ax);
            tht = atan2(cos(phi) * ax + sin(phi) * ay, az);
            psi = atan2(cos(phi) * sy - sin(phi) * sx, cos(phi) * ny - sin(phi) * nx);
        case 2 % ZYX
            phi = atan2(sy, sx);
            tht = atan2(-sz, cos(phi) * sx + sin(phi) * sy);
            psi = atan2(-cos(phi) * ay + sin(phi) * ax, cos(phi) * ny - sin(phi) * nx);
        case 3 % XYZ
            phi = atan2(-ay, az);
            tht = atan2(ax, cos(phi) * az - sin(phi) * ay);
            psi = atan2(cos(phi) * sy + sin(phi) * sz, cos(phi) * ny + sin(phi) * nz);
        otherwise % ZXZ
            phi = atan2(-ax, ay);
            tht = atan2(sin(phi) * ax - cos(phi) * ay, az);
            psi = atan2(-cos(phi) * nx - sin(phi) * ny, cos(phi) * sx + sin(phi) * sy);
    end

    r_pos = [phi;tht;psi];

end

function [theta,r_axs] = getAxisAngle(R_mat)

    sx = R_mat(1,1);
    sy = R_mat(2,1);
    sz = R_mat(3,1);
    nx = R_mat(1,2);
    ny = R_mat(2,2);
    nz = R_mat(3,2);
    ax = R_mat(1,3);
    ay = R_mat(2,3);
    az = R_mat(3,3);
    
    r_axs = zeros(3,1);

    Ctheta = 0.5*(sx+ny+az-1);
    Stheta = 0.5*sqrt((nz-ay)^2+(ax-sz)^2+(sy-nx)^2);
    theta = atan2(Stheta,Ctheta);
    
    if Ctheta < 0
        r_axs(1) = sign(nz-ay)*sqrt((sx-Ctheta)/(1-Ctheta));
        r_axs(2) = sign(ax-sz)*sqrt((ny-Ctheta)/(1-Ctheta));
        r_axs(3) = sign(sy-nx)*sqrt((az-Ctheta)/(1-Ctheta));
    elseif Stheta ~= 0
        r_axs(1) = 0.5*(nz-ay)/Stheta;
        r_axs(2) = 0.5*(ax-sz)/Stheta;
        r_axs(3) = 0.5*(sy-nx)/Stheta;
    end
    
end

function [q_pos,err] = invKinAnalytic(kinPara,TGoal,qPrev,initial)

    err = 0;

    a_i = kinPara.a_i;
    alpha_i = kinPara.alpha_i;
    d_i = kinPara.d_i;
    theta_i_O = kinPara.theta_i_O;
    d_e = kinPara.d_e;
    TI_0 = kinPara.TI_0;
    q_posLim = kinPara.q_posLim;
    
    T0_E = TI_0\TGoal;
    
    p15x = T0_E(1,4) - T0_E(1,3)*(d_i(6) + d_e);
    p15y = T0_E(2,4) - T0_E(2,3)*(d_i(6) + d_e);
%     p15z = T0_E(3,4) - T0_E(3,3)*(d_i(6) + d_e) - d_i(1);
    
    p15xy = sqrt(p15y^2+p15x^2);
    
    if p15x < 0
        alpha1 = pi-atan2(p15y,p15x);
    else
        alpha1 = atan2(p15y,p15x);
    end
    alpha2 = asin(d_i(4)/p15xy);
    
    q1 = alpha1 - alpha2;
%     if q1 < 1e-5 && q1 > -1e-5
%         q1 = 0;
%     end
%     if p15x < 0
%         q1 = -q1;
%     end

    if (abs(qPrev(1) - q1) < abs(qPrev(1) + q1)) && initial == 0
        q1 = -q1;
    end
    q1 = -q1;
    
    if q1 < deg2rad(q_posLim(1,1))
        q1 = deg2rad(q_posLim(1,1));
    elseif q1 > deg2rad(q_posLim(1,2))
        q1 = deg2rad(q_posLim(1,2));
    end

    c1 = cos(q1);
    s1 = sin(q1);
    
%     term1 = T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4)
%     term2 = T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4) - (d_i(6) + d_e)
%     term3 = (T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4)) / (d_i(6) + d_e)
    
%     if vat < 1e-
    q5 = acos((T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/(d_i(6) + d_e));
    q5 = -q5;
    
%     abs(qPrev(5) + q5)
%     abs(qPrev(5) - q5)
%     if (abs(qPrev(5) + q5) <= abs(qPrev(5) - q5)) && initial == 0
%         q5 = -q5;
%     end
%     q5
%     if (abs(qPrev(5) - (2*pi - q5)) < abs(qPrev(5) - q5)) && initial == 0
%         q5 = 2*pi-q5;
%     end
%     if k > 70 && k < 88
%         q5 = -q5;
%     end
    
    if q5 < deg2rad(q_posLim(5,1))
        q5 = deg2rad(q_posLim(5,1));
    elseif q5 > deg2rad(q_posLim(5,2))
        q5 = deg2rad(q_posLim(5,2));
    end
    
    s5 = sin(q5);

    q6 = pi+atan2((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5,(T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5);
    if q6 > pi
        q6 = q6-2*pi;
    end

    if q6 < deg2rad(q_posLim(6,1))
        q6 = deg2rad(q_posLim(6,1));
    elseif q6 > deg2rad(q_posLim(6,2))
        q6 = deg2rad(q_posLim(6,2));
    end

    T0_1 = fwdGeo4Inv(a_i(1),alpha_i(1),d_i(1),theta_i_O(1)+q1);
    T4_5 = fwdGeo4Inv(a_i(5),alpha_i(5),d_i(5),theta_i_O(5)+q5);
    T5_E = fwdGeo4Inv(a_i(6),alpha_i(6),d_i(6)+d_e,theta_i_O(6)+q6);

%     T0_5 = T0_E/T5_E;
%     T0_4 = T0_5/T4_5;
%     T1_4 = T0_1\T0_4;
%     T1_5 = T0_1\T0_5;

    T0_5 = T0_E/T5_E;
    T1_5 = T0_1\T0_5;
    T1_4 = T1_5/T4_5;

    % double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) 
    %             - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1;
    % double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

%     p14x = d_i(5)*( sin(q6) * (T0_E(1,1)*cos(q1) + T0_E(2,1)*sin(q1)) + cos(q6) * (T0_E(1,2)*cos(q1) + T0_E(2,2)*sin(q1)) ) ...
%          - d_i(6)*( T0_E(1,3)*cos(q1) + T0_E(2,3)*sin(q1) ) + T0_E(1,4)*cos(q1) + T0_E(2,4)*sin(q1)
%     p13y = T0_E(3,4) - d_i(1) - d_i(6)*T0_E(3,3) + d_i(5)*(T0_E(3,2)*cos(q6) + T0_E(3,1)*sin(q6))

    p14x = T1_4(1,4);
    p14z = T1_4(3,4);
    p14xzsq = p14x^2 + p14z^2;
    p14xz = sqrt(p14xzsq);

    p15x = T1_5(1,4);
    p15z = T1_5(3,4);
    p15xzsq = p15x^2 + p15z^2;

    % double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
    gamma = acos((a_i(3)^2 - a_i(4)^2 + p14xzsq) / (2*a_i(3)*p14xz));
    beta = acos((a_i(4)^2 - a_i(3)^2 + p14xzsq) / (2*a_i(4)*p14xz));

    if imag(gamma) > 0 || imag(beta) > 0
        q_pos = qPrev;
        err = 1;
        return
    end
%     atan2(p14z,p14x)
%     if atan2(p14z,p14x) < 0
%         
%         disp('nooo')
%         
%         if term2 < -5e-4
%             q5 = -q5
%         end
%         
%         if q5 < deg2rad(q_posLim(5,1))
%             q5 = deg2rad(q_posLim(5,1));
%         elseif q5 > deg2rad(q_posLim(5,2))
%             q5 = deg2rad(q_posLim(5,2));
%         end
% 
%         s5 = sin(q5);
% 
%         q6 = pi+atan2((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5,(T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5);
%         if q6 > pi
%             q6 = q6-2*pi;
%         end
% 
%         if q6 < deg2rad(q_posLim(6,1))
%             q6 = deg2rad(q_posLim(6,1));
%         elseif q6 > deg2rad(q_posLim(6,2))
%             q6 = deg2rad(q_posLim(6,2));
%         end
% 
%         T0_1 = fwdGeo4Inv(a_i(1),alpha_i(1),d_i(1),theta_i_O(1)+q1);
%         T4_5 = fwdGeo4Inv(a_i(5),alpha_i(5),d_i(5),theta_i_O(5)+q5);
%         T5_E = fwdGeo4Inv(a_i(6),alpha_i(6),d_i(6)+d_e,theta_i_O(6)+q6);
% 
%     %     T0_5 = T0_E/T5_E;
%     %     T0_4 = T0_5/T4_5;
%     %     T1_4 = T0_1\T0_4;
%     %     T1_5 = T0_1\T0_5;
% 
%         T0_5 = T0_E/T5_E;
%         T1_5 = T0_1\T0_5;
%         T1_4 = T1_5/T4_5;
% 
%         % double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) 
%         %             - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1;
%         % double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);
% 
%     %     p14x = d_i(5)*( sin(q6) * (T0_E(1,1)*cos(q1) + T0_E(2,1)*sin(q1)) + cos(q6) * (T0_E(1,2)*cos(q1) + T0_E(2,2)*sin(q1)) ) ...
%     %          - d_i(6)*( T0_E(1,3)*cos(q1) + T0_E(2,3)*sin(q1) ) + T0_E(1,4)*cos(q1) + T0_E(2,4)*sin(q1)
%     %     p13y = T0_E(3,4) - d_i(1) - d_i(6)*T0_E(3,3) + d_i(5)*(T0_E(3,2)*cos(q6) + T0_E(3,1)*sin(q6))
% 
%         p14x = T1_4(1,4);
%         p14z = T1_4(3,4);
%         p14xzsq = p14x^2 + p14z^2;
%         p14xz = sqrt(p14xzsq);
% 
%         p15x = T1_5(1,4);
%         p15z = T1_5(3,4);
%         p15xzsq = p15x^2 + p15z^2;
% 
%         % double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
%         gamma = acos((a_i(3)^2 - a_i(4)^2 + p14xzsq) / (2*a_i(3)*p14xz));
%         beta = acos((a_i(4)^2 - a_i(3)^2 + p14xzsq) / (2*a_i(4)*p14xz));
% 
%         if imag(gamma) > 0 || imag(beta) > 0
%             q_pos = qPrev;
%             err = 1;
%             return
%         end
%     
%     end
%        p14x 
    if p14x < 0
        phi = pi-atan2(p14z,p14x);
        q2 = gamma+phi-pi/2;
    else
        phi = atan2(p14z,p14x);
        q2 = gamma+phi-pi/2;
        q2 = -q2;
    end

%     if (abs(qPrev(2) + q2) < abs(qPrev(2) - q2)) && initial == 0
%         q2 = -q2;
%     elseif (abs(qPrev(2) - (q2 + 2*pi)) < abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 + 2*pi;
%     elseif (abs(qPrev(2) - (q2 - 2*pi)) < abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 - 2*pi;
%     elseif  (abs(qPrev(2) - (q2 + pi)) > abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 + pi;
%     elseif  (abs(qPrev(2) - (q2 - pi)) > abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 - pi;
%     elseif  (abs(qPrev(2) - (q2 + pi/2)) > abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 + pi/2;
%     elseif  (abs(qPrev(2) - (q2 - pi/2)) > abs(qPrev(2) - q2)) && initial == 0
%         q2 = q2 - pi/2;
%     end
    if (abs(qPrev(2) - (q2 - 2*pi)) < abs(qPrev(2) - q2)) && initial == 0
        q2 = q2 - 2*pi;
    end
    if (abs(qPrev(2) - (q2 - 2*pi)) < abs(qPrev(2) - q2)) && initial == 0
        q2 = q2 - 2*pi;
    end

    if q2 < deg2rad(q_posLim(2,1))
        q2 = deg2rad(q_posLim(2,1));
    elseif q2 > deg2rad(q_posLim(2,2))
        q2 = deg2rad(q_posLim(2,2));
    end
        
    q3 = gamma + beta;
    if p14x < 0
        q3 = -q3;
    end
    
    if q3 < deg2rad(q_posLim(3,1))
        q3 = deg2rad(q_posLim(3,1));
    elseif q3 > deg2rad(q_posLim(3,2))
        q3 = deg2rad(q_posLim(3,2));
    end

    epsilon = acos((p14xzsq + d_i(5)^2 - p15xzsq) / (2*d_i(5)*p14xz));
    
    q4 = pi-beta-epsilon;
    if p14x < 0
        q4 = -q4;
    end
    
    if (abs(qPrev(4) - (q4 + pi/2)) < abs(qPrev(4) - q4)) && initial == 0
        q4 = q4 + pi/2;
    end
    
    if q4 < deg2rad(q_posLim(4,1))
        q4 = deg2rad(q_posLim(4,1));
    elseif q4 > deg2rad(q_posLim(4,2))
        q4 = deg2rad(q_posLim(4,2));
    end

%     qPrev
    q_pos = [q1;q2;q3;q4;q5;q6];
    
    function Th_i = fwdGeo4Inv(a_i,alpha_i,d_i,theta_i)

        Rot_x_h = Rot_x(alpha_i);
        Trn_x_h	= Trn_x(a_i);
        Rot_z_i = Rot_z(theta_i);
        Trn_z_i = Trn_z(d_i);

        Th_i = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;

        function output = Rot_x(input)
            output = [	1	0           0           0
                        0	cos(input)  -sin(input)	0
                        0	sin(input)	cos(input)	0
                        0	0           0           1 ];
        end

        function output = Rot_z(input)
            output = [  cos(input)	-sin(input)	0	0
                        sin(input)	 cos(input)	0	0
                        0            0          1	0
                        0            0          0	1 ];
        end

        function output = Trn_x(input)
            output = [  1 0 0 input
                        0 1 0 0
                        0 0 1 0
                        0 0 0 1     ];
        end

        function output = Trn_z(input)
            output = [  1 0 0 0
                        0 1 0 0
                        0 0 1 input
                        0 0 0 1     ];
        end
    end
end

function [q_pos,err] = invKinAnaBasic(kinPara,TGoal,qPrev)

    err = 0;

    a_i = kinPara.a_i;
    alpha_i = kinPara.alpha_i;
    d_i = kinPara.d_i;
    theta_i_O = kinPara.theta_i_O;
    d_e = kinPara.d_e;
    TI_0 = kinPara.TI_0;
%     q_posLim = kinPara.q_posLim;
    
    T0_E = TI_0\TGoal;
    
    p15x = T0_E(1,4) - T0_E(1,3)*(d_i(6) + d_e);
    p15y = T0_E(2,4) - T0_E(2,3)*(d_i(6) + d_e);
%     p15z = T0_E(3,4) - T0_E(3,3)*(d_i(6) + d_e) - d_i(1);
    
    p15xy = sqrt(p15y^2+p15x^2);
    
    if p15x < 0
        alpha1 = atan2(p15y,p15x)-pi;
    else
        alpha1 = -atan2(p15y,p15x);
    end
    if d_i(4)/p15xy > 1
        q_pos = qPrev;
        err = 1;
        return
    end
%     alpha1
    alpha2 = asin(d_i(4)/p15xy);
    
    q1 = alpha2 + alpha1;
    
    if abs(qPrev(1) - (q1 + 2*pi)) < abs(qPrev(1) - q1)
        q1 = q1 + 2*pi;
    elseif abs(qPrev(1) - (q1 - 2*pi)) < abs(qPrev(1) - q1)
        q1 = q1 - 2*pi;
    end
    
    c1 = cos(q1);
    s1 = sin(q1);
    
    if (T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/(d_i(6) + d_e) > 1
        q_pos = qPrev;
        err = 1;
        return
    end

    q5 = acos((T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/(d_i(6) + d_e));
    q5 = -q5;
    
    s5 = sin(q5);

    q6 = pi+atan2((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5,(T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5);
    
    if abs(qPrev(6) - (q6 + 2*pi)) < abs(qPrev(6) - q6)
        q6 = q6 + 2*pi;
    elseif abs(qPrev(6) - (q6 - 2*pi)) < abs(qPrev(6) - q6)
        q6 = q6 - 2*pi;
    end
    
    T0_1 = fwdGeo4Inv(a_i(1),alpha_i(1),d_i(1),theta_i_O(1)+q1);
    T4_5 = fwdGeo4Inv(a_i(5),alpha_i(5),d_i(5),theta_i_O(5)+q5);
    T5_E = fwdGeo4Inv(a_i(6),alpha_i(6),d_i(6)+d_e,theta_i_O(6)+q6);

    T0_5 = T0_E/T5_E;
    T1_5 = T0_1\T0_5;
    T1_4 = T1_5/T4_5;

    p14x = T1_4(1,4);
    p14z = T1_4(3,4);
    p14xzsq = p14x^2 + p14z^2;
    p14xz = sqrt(p14xzsq);

    p15x = T1_5(1,4);
    p15z = T1_5(3,4);
    p15xzsq = p15x^2 + p15z^2;

    % double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
    gamma = acos((a_i(3)^2 - a_i(4)^2 + p14xzsq) / (2*a_i(3)*p14xz));
    beta = acos((a_i(4)^2 - a_i(3)^2 + p14xzsq) / (2*a_i(4)*p14xz));

    if imag(gamma) > 0 || imag(beta) > 0
        q_pos = qPrev;
        err = 1;
        return
    end

    if p14x < 0
        phi = pi-atan2(p14z,p14x);
        q2 = gamma+phi-pi/2;
    else
        phi = atan2(p14z,p14x);
        q2 = gamma+phi-pi/2;
        q2 = -q2;
    end
 
    if abs(qPrev(2) - (q2 + 2*pi)) < abs(qPrev(2) - q2)
        q2 = q2 + 2*pi;
    elseif abs(qPrev(2) - (q2 - 2*pi)) < abs(qPrev(2) - q2)
        q2 = q2 - 2*pi;
    end
    
    q3 = gamma + beta;
    if p14x < 0
        q3 = -q3;
    end
    
    epsilon = acos((p14xzsq + d_i(5)^2 - p15xzsq) / (2*d_i(5)*p14xz));
    
    q4 = pi-beta-epsilon;
    if p14x < 0
        q4 = -q4;
    end
    
    q_pos = [q1;q2;q3;q4;q5;q6];
    
    function Th_i = fwdGeo4Inv(a_i,alpha_i,d_i,theta_i)

        Rot_x_h = Rot_x(alpha_i);
        Trn_x_h	= Trn_x(a_i);
        Rot_z_i = Rot_z(theta_i);
        Trn_z_i = Trn_z(d_i);

        Th_i = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;

        function output = Rot_x(input)
            output = [	1	0           0           0
                        0	cos(input)  -sin(input)	0
                        0	sin(input)	cos(input)	0
                        0	0           0           1 ];
        end

        function output = Rot_z(input)
            output = [  cos(input)	-sin(input)	0	0
                        sin(input)	 cos(input)	0	0
                        0            0          1	0
                        0            0          0	1 ];
        end

        function output = Trn_x(input)
            output = [  1 0 0 input
                        0 1 0 0
                        0 0 1 0
                        0 0 0 1     ];
        end

        function output = Trn_z(input)
            output = [  1 0 0 0
                        0 1 0 0
                        0 0 1 input
                        0 0 0 1     ];
        end
    end
end