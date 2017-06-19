%load files
clear all
clc
load('yawrateid_processed.mat')
do=0;
start=1;
if do
cl=input('enter "y" if want to clear data');
if cl=='y'
    count=0;
    clearvars steady_yawrate steady_longvelocity steady_steer tindex
    disp('data cleared')
end
for i=start:72
    figure(1)
    plot(processeddata(i).mocap.time,processeddata(i).mocap.longvelocity)
     hold on
    plot(processeddata(i).mocap.time,processeddata(i).mocap.yawrate)
    xlabel('time')
    xlim([0,10])
    ylabel('long v,yaw rate')
    ylim([-2.5,2.5])
    ax=gca;
    ax.XGrid='on';
    ax.YGrid='on';
    hold off
    disp(['Trial: ',num2str(i)])
    ts=input('enter start time for constant v,w: ');
    te=input('enter start time for constant v,w: ');
    if (isnumeric(ts) && ts~=0)|| (isnumeric(te) && te~=0)
        count=count+1;
        is=find(processeddata(i).mocap.time>=ts,1,'first');
        ie=find(processeddata(i).mocap.time<=te,1,'last');
        steady_yawrate(count)=mean(processeddata(i).mocap.yawrate(is:ie));
        steady_longvelocity(count)=mean(processeddata(i).mocap.longvelocity(is:ie));
        is=find(processeddata(i).input.time>=ts,1,'first');
        ie=find(processeddata(i).input.time<=te,1,'last');
        steady_steer(count)=mean(processeddata(i).input.steering(is:ie));
        tindex(count)=i;
    else
        disp(['skipping trial: ',num2str(i)])
        
    end
end
end
clearvars te ts ie is
clearvars i
clearvars ax
save('yawrateid_processed.mat')
%%
close all
figure
plot(steady_steer,atan2(steady_yawrate*.29,steady_longvelocity),'*')
wheelangle=polyval(kinematic_linear_fit.coeff,(steady_steer-1590)/225.09);
estyawrate=tan(wheelangle).*steady_longvelocity/.29;
figure
plot(steady_yawrate,estyawrate,'*')
%%

d=.1;
m=2.759;
g=9.81;
l=.29;
cf=5000;
cr=500;
kus=(m*g*d/(l*cf)-m*g*(l-d)/(l*cr));
vcr=sqrt(-l*g/(kus))
figure
u1=steady_steer-1590;
delta=-3.67478776615971e-10*u1.^3-4.59218815521785e-07*u1.^2-0.00126740525229302*u1-0.0204632799650850;
plot(steady_longvelocity,steady_yawrate.*(delta).^-1,'*')
estyr=delta.*steady_longvelocity.*((l+kus/g*steady_longvelocity.^2).^-1);
plot(steady_steer,steady_yawrate.*(l+kus/g*steady_longvelocity.^2).*(steady_longvelocity.^-1),'k*')



