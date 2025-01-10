function [q_pos,err] = invKinAnaBasic2(kinPara,TGoal,qPrev)

    err = 0;

    a_i = kinPara.a_i;
    alpha_i = kinPara.alpha_i;
    d_i = kinPara.d_i;
    theta_i_O = kinPara.theta_i_O;
    d_e = kinPara.d_e;
    TI_0 = kinPara.TI_0;
%     q_posLim = kinPara.q_posLim;
    
%     TI_0
%     TGoal

    T0_E = TI_0\TGoal;
    
    %% calculate q1
    p15x = T0_E(1,4) - T0_E(1,3)*(d_i(6) + d_e);
    p15y = T0_E(2,4) - T0_E(2,3)*(d_i(6) + d_e);
%     p15z = T0_E(3,4) - T0_E(3,3)*(d_i(6) + d_e) - d_i(1);
    
    p15xy = sqrt(p15y^2+p15x^2)
    
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
    
    q1 = alpha1 + alpha2;
    
    q1cnt = 0;
    q1temp = -alpha1-alpha2;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 1;
        q1 = q1temp;
    end
    q1temp = alpha1+alpha2+2*pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 2;
        q1 = q1temp;
    end
    q1temp = alpha1+alpha2-2*pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 3;
        q1 = q1temp;
    end
    q1temp = -alpha1+alpha2-pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 4;
        q1 = q1temp;
    end
    q1temp = alpha1-alpha2+pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 5;
        q1 = q1temp;
    end
    q1temp = -alpha1+alpha2+pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 6;
        q1 = q1temp;
    end
    q1temp = alpha1-alpha2-pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 7;
        q1 = q1temp;
    end
    q1temp = -alpha1-alpha2-2*pi;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 8;
        q1 = q1temp;
    end
    q1temp = 3*pi+alpha1-alpha2;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 9;
        q1 = q1temp;
    end
    q1temp = 2*pi-alpha1-alpha2;
    if abs(qPrev(1) - q1temp) < abs(qPrev(1) - q1)
        q1cnt = 10;
        q1 = q1temp;
    end
    q1cnt
    
    %% calculate q5
    c1 = cos(q1);
    s1 = sin(q1);
    
    if (T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/(d_i(6) + d_e) > 1
        q_pos = qPrev;
        err = 1;
        return
    end

    q5 = acos((T0_E(2,4)*c1 - T0_E(1,4)*s1 - d_i(4))/(d_i(6) + d_e));

    q5orig = q5;

    q5cnt = 0;
    if abs(qPrev(5) + q5) < abs(qPrev(5) - q5)
        q5cnt = 1;
        q5 = -q5;
    end
    q5temp = q5orig-2*pi;
    if abs(qPrev(5) - q5temp) < abs(qPrev(5) - q5)
        q5cnt = 2;
        q5 = q5temp;
    end
    q5temp = -q5orig+2*pi;
    if abs(qPrev(5) - q5temp) < abs(qPrev(5) - q5)
        q5cnt = 3;
        q5 = q5temp;
    end
    q5cnt

    %% calculate q6
    s5 = sin(q5);

    if imag((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5) ~= 0 || imag((T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5) ~= 0
        q_pos = qPrev;
        err = 1;
        return
    end

    q6 = pi+atan2((T0_E(2,2)*c1 - T0_E(1,2)*s1)/s5,(T0_E(1,1)*s1 - T0_E(2,1)*c1)/s5);
    
    q6cnt = 0;
    if abs(qPrev(6) - (q6 + 2*pi)) < abs(qPrev(6) - q6)
        q6cnt = 1;
        q6 = q6 + 2*pi;
    elseif abs(qPrev(6) - (q6 - 2*pi)) < abs(qPrev(6) - q6)
        q6cnt = 2;
        q6 = q6 - 2*pi;
    end
    q6cnt
    
    %% calculate q2
    T0_1 = fwdGeo4Inv(a_i(1),alpha_i(1),d_i(1),theta_i_O(1)+q1);
    T4_5 = fwdGeo4Inv(a_i(5),alpha_i(5),d_i(5),theta_i_O(5)+q5);
    T5_E = fwdGeo4Inv(a_i(6),alpha_i(6),d_i(6)+d_e,theta_i_O(6)+q6);

    T0_5 = T0_E/T5_E;
    T1_5 = T0_1\T0_5;
    T1_4 = T1_5/T4_5;

%     tic
%     T0_E/T5_E
%     toc
%     tic
%     T0_E*inv(T5_E)
%     toc
%     tic
%     T0_E*invT(T5_E)
%     toc

    p14x = T1_4(1,4)
    p14z = T1_4(3,4)
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

    phi = atan2(p14z,p14x);

%     if p14x < 0
%         phi = pi-atan2(p14z,p14x);
%         q2 = gamma+phi-pi/2;
%     else
%         phi = atan2(p14z,p14x);
%         q2 = -(gamma+phi-pi/2);
%     end
        
%     rad2deg(qPrev(2))
%     rad2deg(q2)
    q2cnt = 0;
    q2 = gamma+phi;
%     rad2deg(q2temp)
    q2temp = gamma+phi + pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 1;
        q2 = q2temp;
    end
    q2temp = gamma+phi - pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 2;
        q2 = q2temp;
    end
    q2temp = gamma+phi + 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 3;
        q2 = q2temp;
    end
    q2temp = gamma+phi - 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 4;
        q2 = q2temp;
    end
    q2temp = gamma+phi + 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 5;
        q2 = q2temp;
    end
    q2temp = gamma+phi - 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 6;
        q2 = q2temp;
    end
    q2temp = gamma+phi + pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 7;
        q2 = q2temp;
    end
    q2temp = gamma+phi - pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 8;
        q2 = q2temp;
    end
    q2temp = gamma+phi + 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 9;
        q2 = q2temp;
    end
    q2temp = gamma+phi - 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 10;
        q2 = q2temp;
    end
    q2temp = gamma+phi + 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 11;
        q2 = q2temp;
    end
    q2temp = gamma+phi - 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 12;
        q2 = q2temp;
    end

    q2temp = gamma-phi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 13;
        q2 = q2temp;
    end
    q2temp = gamma-phi + pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 14;
        q2 = q2temp;
    end
    q2temp = gamma-phi - pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 15;
        q2 = q2temp;
    end
    q2temp = gamma-phi + 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 16;
        q2 = q2temp;
    end
    q2temp = gamma-phi - 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 17;
        q2 = q2temp;
    end
    q2temp = gamma-phi + 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 18;
        q2 = q2temp;
    end
    q2temp = gamma-phi - 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 19;
        q2 = q2temp;
    end
    q2temp = gamma-phi + pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 20;
        q2 = q2temp;
    end
    q2temp = gamma-phi - pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 21;
        q2 = q2temp;
    end
    q2temp = gamma-phi + 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 22;
        q2 = q2temp;
    end
    q2temp = gamma-phi - 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 23;
        q2 = q2temp;
    end
    q2temp = gamma-phi + 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 24;
        q2 = q2temp;
    end
    q2temp = gamma-phi - 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 25;
        q2 = q2temp;
    end

    q2temp = -gamma+phi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 26;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 27;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 28;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 29;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 30;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 31;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 32;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 33;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 34;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 35;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 36;
        q2 = q2temp;
    end
    q2temp = -gamma+phi + 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 37;
        q2 = q2temp;
    end
    q2temp = -gamma+phi - 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 38;
        q2 = q2temp;
    end

    q2temp = -gamma-phi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 39;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 40;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 41;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 42;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - 2*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 43;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 44;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - 3*pi;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 45;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 46;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 47;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 48;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - 3*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 49;
        q2 = q2temp;
    end
    q2temp = -gamma-phi + 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 50;
        q2 = q2temp;
    end
    q2temp = -gamma-phi - 5*pi/2;
%     rad2deg(q2temp)
    if abs(qPrev(2) - q2temp) < abs(qPrev(2) - q2)
        q2cnt = 51;
        q2 = q2temp;
    end
    q2cnt
    
    %% calculate q3
    q3 = gamma + beta;
    if p14x < 0
        q3 = -q3;
    end
    
    q3cnt = 0;
    if abs(qPrev(3) + q3) < abs(qPrev(3) - q3)
        q3cnt = 1;
        q3 = -q3;
    end
    q3temp = -2*pi+gamma+beta;
    if abs(qPrev(3) - q3temp) < abs(qPrev(3) - q3)
        q3cnt = 2;
        q3 = q3temp;
    end
    q3temp = 2*pi-gamma-beta;
    if abs(qPrev(3) - q3temp) < abs(qPrev(3) - q3)
        q3cnt = 3;
        q3 = q3temp;
    end
    q3cnt

    %% calculate q4
    epsilon = acos((p14xzsq + d_i(5)^2 - p15xzsq) / (2*d_i(5)*p14xz));
    
    q4 = pi-beta-epsilon;
    if p14x < 0
        q4 = -q4;
    end
    
    q4orig = q4;

%     rad2deg(qPrev(4))
%     rad2deg(q4)
    q4cnt = 0;
    q4temp = pi/2+beta-epsilon;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 1;
        q4 = q4temp;
    end
    q4temp = pi/2-beta-epsilon;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 2;
        q4 = q4temp;
    end
    q4temp = -pi/2-q4orig;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 3;
        q4 = q4temp;
    end
    q4temp = -pi/2+q4orig;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 4;
        q4 = q4temp;
    end
    q4temp = pi/2+epsilon-beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 5;
        q4 = q4temp;
    end
    q4temp = 3*pi/2-q4orig;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 6;
        q4 = q4temp;
    end
    q4temp = 3*pi/2+q4orig;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 7;
        q4 = q4temp;
    end
    q4temp = -3*pi/2+epsilon-beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 8;
        q4 = q4temp;
    end
    q4temp = -3*pi/2-epsilon-beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 9;
        q4 = q4temp;
    end
    q4temp = -3*pi/2-epsilon+beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 10;
        q4 = q4temp;
    end
    q4temp = 5*pi/2+epsilon-beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 11;
        q4 = q4temp;
    end
    q4temp = 5*pi/2-epsilon+beta;
%     rad2deg(q4temp)
    if abs(qPrev(4) - q4temp) < abs(qPrev(4) - q4)
        q4cnt = 12;
        q4 = q4temp;
    end
    q4cnt

    %% get all together
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

    function Tb_a = invT(Ta_b)
        Ra_bTrn = Ta_b(1:3,1:3)';
        Tb_a = [Ra_bTrn,-Ra_bTrn*Ta_b(1:3,4);0 0 0 1];
    end
end