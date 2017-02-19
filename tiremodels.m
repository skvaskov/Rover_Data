clear all
load('smoothdata100.mat')

trial=processeddata25;
f=40;
ay=trial(18,1:f);
vx=trial(8,1:f);
r=trial(26,1:f);
time=trial(1,1:f);
vy=trial(10,1:f);

slipar=zeros(1,f);
fyr=zeros(1,f);

m=2.47+.289;
lf=.2;
l=.29;

for i=1:f
    slipar(i)=atan2(r(i)*(l-lf)-vy(i),vx(i));
    fyr(i)=m*lf/l*ay(i);
    slope=fyr(i)/slipar(i)
end

plot(abs(slipar),abs(fyr),'.')


