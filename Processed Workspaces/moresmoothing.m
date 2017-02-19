%remove outliers from sideslip data
clear all
load('smoothdata100post.mat')
trial=processeddata25;
time=trial(1,:);
datasize=length(time);

figure
subplot(2,1,1)
plot(trial(21,:))
hold on
plot(trial(22,:))

subplot(2,1,2)
plot(trial(27,:))
hold on
plot(trial(28,:))

%%
index=200;
replace=trial(21,index)*ones(1,datasize-index);
trial(21,:)=[trial(21,1:index) replace];
sm=smooth(trial(21,:))';

figure
subplot(2,1,1)
plot(time,trial(21,:))
hold on
plot(time,sm')

dss=zeros(1,datasize);
for i=1:datasize-1
    dss(i)=(sm(i+1)-sm(i))/(time(i+1)-time(i));
end
dss(datasize)=dss(datasize-1);
dsssm=smooth(dss,25,'sgolay',2)';

subplot(2,1,2)
plot(time,dss)
hold on
plot(time,dsssm)

%%
trial(22,:)=sm;
trial(27,:)=dss;
trial(28,:)=dsssm;

processeddata25=trial;
clearvars -except processeddata*
save('smoothdata100post.mat')

