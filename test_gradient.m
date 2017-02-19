v = rand(size(user.z0)) ;

tic
gNum = numericJacobian(@user.equalityConstraints,1,v)' ;
toc

tic
[~,gAnl] = user.equalityConstraints(v) ;
toc

disp(['Numeric vs. analytic gradients: ', num2str(norm(gAnl - gNum))]) ;
