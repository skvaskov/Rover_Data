load('smoothdata100postalternative.mat')
repeat8=[processeddata8(:,1:250) processeddata8(:,250)+processeddata8(:,2:250)];

plot(repeat8(1,:),repeat8(4,:))
hold on
plot(repeat8(1,:),repeat8(6,:))
plot(repeat8(1,:),1/100*repeat8(29,:))

%%
save('smoothdata100postalternative.mat')
a='saved'