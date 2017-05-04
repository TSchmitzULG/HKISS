function ATransform = volterraTransform(nbKernels)

ATransform = zeros(nbKernels);

for u = 1 : nbKernels
    for v = 1: nbKernels
        if (mod(u+v,2)==0 && v>=u )                 
            ATransform(u,v) = (((-1)^((1-u)/2))/(2^(v-1)))*nchoosek(v,(v-u)/2) ;
        end;
    end;
end;
ATransform = inv(ATransform);