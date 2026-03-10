function OLTF = closedLoopToPlant(CLTF,Kp,reductionOrder)
    Gtf = tf(CLTF); 
    if nargin>2
        OLTF = balred(Gtf,reductionOrder);   % 4. dereceli hale indir
    else
        OLTF = Gtf / (Kp * (1 - Gtf));
    end
end
