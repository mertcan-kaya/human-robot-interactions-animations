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

    function output = Trn_z(input)
        output = [  1 0 0 0
                    0 1 0 0
                    0 0 1 input
                    0 0 0 1     ];
    end

end