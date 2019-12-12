function curve = patheval(njoints,phasedif,wvel,amplitude,cons,t)
    A = [];
    for i = 1:njoints
        A = [A ; amplitude*sin(wvel*t + (i-1)*phasedif) + cons];
    end
    curve = A;
    
% This function calculates serpenoid curve for every joint angle variable
% and given the parameters njoints (number of joints in the system),
% phasedif (phase difference of joint's angles with regards to next and
% last joint),wvel (Angular velocity of every joint),amplitude (Serpenoid's
% curve amplitude) and cons (Offset of the serpenoid curve)