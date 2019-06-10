function [Val,Ister,Dir] = StopSimulation(t,st)
x = st(1);
y = st(2);
Val(1) = (12^2+12^2) - (x^2+y^2);
Ister(1) = 1;
Dir(1) = 0;