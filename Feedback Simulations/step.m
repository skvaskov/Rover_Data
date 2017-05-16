function [out] = step(t,outarray,stepsize)
tarray=cumsum(stepsize);
out=zeros(length(t),1);
for i=1:length(t)
    
idx=find(t(i)<=tarray,1);
out(i)=outarray(idx);
end

end

