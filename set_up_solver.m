clear
clc

datastruct = load('smoothdata100imutime.mat') ;
tarray=[17];
data = {} ;
input = {} ;
t = {} ;
start=1;
done=200;

for idx = 1:length(tarray)
    trial = datastruct.(['processeddata' num2str(tarray(idx))]) ;
    data{idx} = trial([10,26],start:done);
    input{idx} = [steeringmodel(trial(32,(start:done)));trial(8,start:done)];
    t{idx} = trial(1,start:done)-trial(1,start) ; 
end

% %add alternative trials into solver
% posshift=[1,2,3,4,5];
% for j=1:length(posshift)
% for i=1:length(tarray)
%     trial = datastruct.(['pddelayed' num2str(tarray(i))]);
%     data{idx+i}= [trial(4,:)+posshift(j);trial(6,:)];
%     input{idx+i} = 1/100*trial(29,:);
%     t{idx+i} = trial(1,:); 
% end
% idx=idx+i;
% end
% 
% idx=idx+i;
% posshift=[1,2,3,4,5];
% for j=1:length(posshift)
% for i=1:length(tarray)
%     trial = datastruct.(['repeat' num2str(tarray(i))]);
%     data{idx+i}= [trial(4,:)+posshift(j);trial(6,:)];
%     input{idx+i} = 1/100*trial(29,:);
%     t{idx+i} = trial(1,:); 
% end
% idx=idx+i;
% end

%masses of rover components: rover 2.470 kg, floureon battery 0.289 kg, lrp
%battery 0.257 kg, traxx battery 0.260 kg
%esitmate for the lower bound of lf: 15.85

fdyn = @lygerosreduced;
%p = [Iz lf l ca p1]' ;
%guesses
l=.29;
lf=.18;
m=2.759;
w=.3;
Izub=2.759/12*(l^2+w^2);
Izg=2.759/12*l^2;
lflb=.1585;

p0 =[2.759,1,1.6,2.9,1,1]';

plb=[m-.01;0;lflb*10;10*(l-.01);0;.8];
pub=[m+.01;Izub*1000;l*10;10*(l+.01);1000;1.2];
% p0=[.0001;.01;.01;.01;1;.5;.119;.64;5];
%  plb=[0;0;0;0;0;0;.118;0.6;0];
%  pub=[1;1;1;1;10;1;.12;0.7;10];
 zlb=-100*ones(1,2)';
% %zlb=[-.01;-2;-pi/2;-5;-5];
 zub=100*ones(1,2);
%zub=[5;2;pi/2;5;5];
s=[1;1];

user = nonlinearModelFit(fdyn,t,data,input,p0,'pl',plb,'pu',pub) ;

