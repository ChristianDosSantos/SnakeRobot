function curve = pathcurve(njoints,phase,wvel,amplitude,cons,t)
    A = [];
    for i = 1:njoints
        A = symfun([A ; amplitude*sin(wvel*t + (i-1)*phase) + cons],t);
    end
    curve = A;
    
