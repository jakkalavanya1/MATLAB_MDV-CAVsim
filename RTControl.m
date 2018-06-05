function[aa,bb,cc,dd]= RTControl(to,tf,xo,xf,vo,vf)

    % This function allows finding a set of constants to control each vehicle 
    % on the road

    A_mtx=[to^3/6 to^2/2 to 1; to^2/2 to 1 0; tf^3/6 tf^2/2 tf 1; tf^2/2 tf 1 0];
    Y_mtx=[xo; vo; xf; vf];

    A_aux=A_mtx'*A_mtx;
    X_mtx=pinv(A_aux)*A_mtx'*Y_mtx;

    aa=X_mtx(1);
    bb=X_mtx(2);
    cc=X_mtx(3);
    dd=X_mtx(4);
end