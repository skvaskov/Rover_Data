function [CPos,trans,heading,pitch_angle,roll_angle,Rot_global,yaw_step,pitch_step,roll_step,Rot,quart,mocaptime,lrms]=rigidbodyfit(methodname,markerdata,markerorder)
    %vectors pointing from each marker to center of rover
    rc(:,5)=[-.01158+.47/2;.178-.345/2;0]; %RC
    rc(:,6)=[-.01442+.47/2;-.011+.345/2;0]; %RR
    rc(:,7)=[.238-.47/2;-.01083+.345/2;0]; %SR
    rc(:,8)=[.01314-.47/2;-.01417+.345/2;0]; %FR
    rc(:,2)=[.01890-.47/2;.01602-.345/2;0];%FL
    rc(:,1)=[.01148-.47/2;.172-.345/2;0];%FC
    rc(:,4)=[-.02088+.47/2;.02038-.345/2;0]; %RL
    rc(:,3)=[.241-.47/2;.01346-.345/2;0];%SL
%reorder marker data
sz=size(markerorder);
markernums=zeros(sz);
for i=1:length(markerorder)
   for j=1:sz(1)
       if markerorder(i)==string(markerdata(j).name)
           markernums(i)=j;
       end
   end
end
markerdata_reorder=markerdata(markernums);
% get center positon and heading for rigid body model of 8 markers
num=size(markerdata);
num=num(2);
idx=1;
l=1;
%find marker with longest time value and smooth and interpolate the rest of the
%markers to align with it
for i=1:num
    len=length(markerdata_reorder(i).time);
    if len>l
        l=len;
        idx=i;
    end
end
mocaptime=markerdata_reorder(idx).time;
trans=zeros(3,length(mocaptime));
quart=zeros(length(mocaptime),4);
CPos=zeros(3,length(mocaptime));
Rot_global=zeros(3,3,length(mocaptime)); %Rotation matrix from initial frame to present
Rot=zeros(3,3,length(mocaptime));
lrms=zeros(length(mocaptime),1);
heading=zeros(length(mocaptime),1);%yaw angle from initial frame (heading)
pitch_angle=zeros(length(mocaptime),1);%pitch angle from initial frame
roll_angle=zeros(length(mocaptime),1);%roll angle from initial frame
yaw_step=zeros(length(mocaptime),1); %yaw angle between frames
pitch_step=zeros(length(mocaptime),1); %pitch angle between frames
roll_step=zeros(length(mocaptime),1); %roll angle between frames
%get initial position
no_init=1;
while no_init
    for j=1:num 
       if isempty(find(markerdata_reorder(j).time==mocaptime(1),1))
          ei(j)=0;
       else
           ei(j)=find(markerdata_reorder(j).time==mocaptime(1),1);
       end
    end
    I=[];
    rc_i=[];
    for j=1:num
        if ei(j)>0
            I=[I,markerdata_reorder(j).pos(ei(j),:)'];
            rc_i=[rc_i,rc(:,j)];
        end
    end
    szi=size(I);
    if szi(2)>=3
        if methodname=='absor'
            [regParams,~,ErrorStats]=absor(-rc_i,I,'doTrans',1);
            U=regParams.R;
            r=regParams.t;
            quart(1,:)=regParams.q;
            lrms(1)=ErrorStats.errlsq;
        elseif methodname=='Kabsch'
            [U,r,lrms(1)]=Kabsch(-rc_i,I);
        elseif methodname=='rot3dfit'
            [U,r,~,lrms(1)]=rot3dfit(-rc_i',I');
              r=r';
              U=U';
        else
            error('Method (first input) should be: absor, Kabsch, or rot3dfit')
        end
        yaw_step(1)=0;
        pitch_step(1)=0;
        roll_step(1)=0;
        Rot(:,:,1)=eye(3);
        trans(:,1)=r;
        Rot_global(:,:,1)=U;
        heading(1)=atan2(U(2,1),U(1,1));
        pitch_angle(1)=atan2(-U(3,1),sqrt(U(3,2)^2+U(3,3)^2));
        roll_angle(1)=atan2(U(3,2),U(3,3));
        %find center position
        CPos(:,1)=mean(U*rc_i+I,2);
        no_init=0;
    else
        quart(1,:)=[];
        mocaptime(1)=[];
        Rot_global(:,:,1)=[];
        trans(:,1)=[];
        lrms(1)=[];
        yaw_step(1)=[];
        pitch_step(1)=[];
        roll_step(1)=[];
        heading(1)=[];
        pitch_angle(1)=[];
        roll_angle(1)=[];
        CPos(:,1)=[];
    end
    i=2;
while i<=length(mocaptime)
    epi=zeros(num,1);
    eqi=zeros(num,1);
    for j=1:num 
       if isempty(find(markerdata_reorder(j).time==mocaptime(i-1),1))
          epi(j)=0;
       else
           epi(j)=find(markerdata_reorder(j).time==mocaptime(i-1),1);
       end
        if isempty(find(markerdata_reorder(j).time==mocaptime(i),1))
         eqi(j)=0;
       else
           eqi(j)=find(markerdata_reorder(j).time==mocaptime(i),1);
       end
    end
    %construct x,y,z data to be fed into algorithm
    P=[];
    Q=[];
    rc_pq=[];
    for j=1:num
        if epi(j)>0 && eqi(j)>0
            rc_pq=[rc_pq,rc(:,j)];
            P=[P,markerdata_reorder(j).pos(epi(j),:)'];
            Q=[Q,markerdata_reorder(j).pos(eqi(j),:)'];
        end
    end
    sz=size(P);
    if sz(2)>=3       
        if methodname=='absor'
            [regParams,~,ErrorStats]=absor(P,Q,'doTrans',1);
            [regParamsG,~,ErrorStatsG]=absor(-rc_pq,Q,'doTrans',1);
               U=regParams.R;
                r=regParams.t;
                lrms(i)=ErrorStats.errlsq;
                q_2=regParams.q;
                quart(i,:)=[ (q_2(1)*quart(i-1,1))-(q_2(2)*quart(i-1,2))-(q_2(3)*quart(i-1,3))-(q_2(4)*quart(i-1,4)),...
                     (q_2(1)*quart(i-1,2))+(q_2(2)*quart(i-1,1))-(q_2(3)*quart(i-1,4))+(q_2(4)*quart(i-1,3)),...
                     (q_2(1)*quart(i-1,3))+(q_2(2)*quart(i-1,4))+(q_2(3)*quart(i-1,1))-(q_2(4)*quart(i-1,2)),...
                     (q_2(1)*quart(i-1,4))-(q_2(2)*quart(i-1,3))+(q_2(3)*quart(i-1,2))+(q_2(4)*quart(i-1,1))];
        elseif methodname=='Kabsch'
            [U,r,lrms(i)]=Kabsch(P,Q);
        elseif methodname=='rot3dfit'
            [U,r,~,lrms(i)]=rot3dfit(P',Q');
              r=r';
              U=U';
        else
            error('Method (first input) should be: absor, Kabsch, or rot3dfit')
        end
        
        yaw_step(i)=atan2(U(2,1),U(1,1));
        pitch_step(i)=atan2(-U(3,1),sqrt(U(3,2)^2+U(3,3)^2));
        roll_step(i)=atan2(U(3,2),U(3,3));
        Rot(:,:,i)=U;
        trans(:,i)=r;
        U_global=U*Rot_global(:,:,i-1);
        Rot_global(:,:,i)=U_global;
        heading(i)=atan2(U_global(2,1),U_global(1,1));
        pitch_angle(i)=atan2(-U_global(3,1),sqrt(U_global(3,2)^2+U_global(3,3)^2));
        roll_angle(i)=atan2(U_global(3,2),U_global(3,3));
%         heading(i)=atan2(2*(quart(i,1).*quart(i,4)+quart(i,2).*quart(i,3)),1-2*(quart(i,3).^2+quart(i,4).^2));
%         pitch_angle(i)=asin(2*(quart(i,1).*quart(i,3)-quart(i,4).*quart(i,2)));
%         roll_angle(i)=atan2(2*(quart(i,1).*quart(i,2)+quart(i,3).*quart(i,4)),1-2*(quart(i,2).^2+quart(i,3).^2));
        %find center position
        CPos(:,i)=mean(U_global*rc_pq+Q,2);
        i=i+1;
    else
        
        mocaptime(i)=[];
        Rot_global(:,:,i)=[];
        trans(:,i)=[];
        lrms(i)=[];
        yaw_step(i)=[];
        pitch_step(i)=[];
        roll_step(i)=[];
        heading(i)=[];
        pitch_angle(i)=[];
        roll_angle(i)=[];
        CPos(:,i)=[];  
    end
end
CPos=CPos';
trans=trans';
end
