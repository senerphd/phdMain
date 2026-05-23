function zdot=mrac_ode_fn(t,z,Ap,Bp,Am,Bm,P,Gx,Gr,umax,rfunc)
    n=3;
    xp=z(1:n); xm=z(n+1:2*n); Kx=z(2*n+1:3*n); Kr=z(3*n+1);
    r=rfunc(t);
    u=min(max(Kx'*xp+Kr*r,-umax),umax);
    xp_dot=Ap*xp+Bp*u;
    xm_dot=Am*xm+Bm*r;
    e=xp-xm;
    sigma=e'*P*Bp;
    Kx_dot=-Gx*xp*sigma;
    Kr_dot=-Gr*r*sigma;
    zdot=[xp_dot;xm_dot;Kx_dot;Kr_dot];
end
