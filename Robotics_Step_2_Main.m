clc; clear;
syms l1 l2 l3 d1 d2 d3 theta1 theta2 theta3

DH =    [l1 d1 0 0;
        l2 0 0 theta2;
        l3 0 0 theta3];
q = [5, pi/2, pi/2];

bot1 = manipulator(DH);
T = bot1.fkine(q);

