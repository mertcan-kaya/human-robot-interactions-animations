function [a_i,alpha_i,d_i,thetaPlus_i] = kinematicParametersUR3

    pO2 = pi/2;

    A1 = 0; A2 = -pO2; A3 = 0; A4 = 0; A5 = pO2; A6 = -pO2;
    T1 = 0; T2 = 0; T3 = 0; T4 = pi; T5 = 0; T6 = 0;
    
    a1 = 0; a2 = 0; a3 = 0.24365; a4 = 0.21325; a5 = 0; a6 = 0;
    d1 = 0.1519; d2 = 0; d3 = 0; d4 = 0.11235; d5 = 0.08535; d6 = 0.0819;
    
    % General DH
    a_i     = [a1;a2;a3;a4;a5;a6];
    alpha_i = [A1;A2;A3;A4;A5;A6];
    d_i     = [d1;d2;d3;d4;d5;d6];
    thetaPlus_i = [T1;T2;T3;T4;T5;T6];
    
end