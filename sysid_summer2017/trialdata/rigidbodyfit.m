function [Rot_step,Rot_conc,Rot_global,trans_step,trans_global,mocaptime,error_step,error_global]=rigidbodyfit(markerdata,initial,min_mark)
 
num=size(markerdata);
num=num(2);
idx=1;
l=1;
%find marker with longest time value 
for i=1:num
    len=length(markerdata(i).time);
    if len>l
        l=len;
        idx=i;
    end
end
mocaptime=markerdata(idx).time;
trans_step=zeros(3,length(mocaptime));
trans_global=zeros(3,length(mocaptime));
Rot_global=zeros(3,3,length(mocaptime)); %Rotation matrix from initial frame to present
Rot_step=zeros(3,3,length(mocaptime));
Rot_conc=zeros(3,3,length(mocaptime));
error_step=zeros(length(mocaptime),1);
error_global=zeros(length(mocaptime),1);

%get initial position
no_init=1;
while no_init
    for j=1:num 
       if isempty(find(markerdata(j).time==mocaptime(1),1))
          ei(j)=0;
       else
           ei(j)=find(markerdata(j).time==mocaptime(1),1);
       end
    end
    I=[];
    r_i=[];
    for j=1:num
        if ei(j)>0
            I=[I,markerdata(j).pos(ei(j),:)'];
            r_i=[r_i,initial(:,j)];
        end
    end
    szi=size(I);
    if szi(2)>=min_mark(1)
        [regParams,~,ErrorStats]=absor(r_i,I,'doTrans',1);
        Rot_step(:,:,1)=regParams.R;
        Rot_conc(:,:,1)=regParams.R;
        Rot_global(:,:,1)=regParams.R;
        trans_step(:,1)=regParams.t;
        trans_global(:,1)=regParams.t;
        error_step(1)=ErrorStats.errlsq;
        error_global(1)=ErrorStats.errlsq;
        no_init=0;
    else
        mocaptime(1)=[];
        Rot_step(:,:,1)=[];
        Rot_conc(:,:,1)=[];
        Rot_global(:,:,1)=[];
        trans_step(:,1)=[];
        trans_global(:,1)=[];
        error_step(1)=[];
        error_global(1)=[];
    end
end
    i=2;
while i<=length(mocaptime)
    epi=zeros(num,1);
    eqi=zeros(num,1);
    for j=1:num 
       if isempty(find(markerdata(j).time==mocaptime(i-1),1))
          epi(j)=0;
       else
           epi(j)=find(markerdata(j).time==mocaptime(i-1),1);
       end
        if isempty(find(markerdata(j).time==mocaptime(i),1))
         eqi(j)=0;
       else
           eqi(j)=find(markerdata(j).time==mocaptime(i),1);
       end
    end
    %construct x,y,z data to be fed into algorithm
    P=[];
    Q=[];
    r_i=[];
    for j=1:num
        if epi(j)>0 && eqi(j)>0
            r_i=[r_i,initial(:,j)];
            P=[P,markerdata(j).pos(epi(j),:)'];
            Q=[Q,markerdata(j).pos(eqi(j),:)'];
        end
    end
    sz=size(P);
    if sz(2)>=min_mark(2)    
        [regParamsS,~,ErrorStatsS]=absor(P,Q,'doTrans',1);
        [regParamsG,~,ErrorStatsG]=absor(r_i,Q,'doTrans',1);
        Rot_step(:,:,i)=regParamsS.R;
        Rot_conc(:,:,i)=regParamsS.R*Rot_conc(:,:,i-1);
        Rot_global(:,:,i)=regParamsG.R;
        trans_step(:,i)=regParamsS.t;
        trans_global(:,i)=regParamsG.t;
        error_step(i)=ErrorStatsS.errlsq;
        error_global(i)=ErrorStatsG.errlsq;
        i=i+1;
    else
        mocaptime(i)=[];
        Rot_step(:,:,i)=[];
        Rot_conc(:,:,i)=[];
        Rot_global(:,:,i)=[];
        trans_step(:,i)=[];
        trans_global(:,i)=[];
        error_step(i)=[];
        error_global(i)=[];
    end
end
end
