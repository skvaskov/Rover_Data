%% TO DO:
% - expand example tester to include noise and various inputs
% - clean up the folders
% - get rid of old nonlinearModelFit that doesn't do multi trial
% - get rid of old examples file (convert it to the new format)

%% example 1
clear
clc
close all

f = @testFunction1 ;
p = [2; 4] ;
x0 = [1, 0, -1;
     -1, -1, -1] ;
p0 = [3; 5] ;
T = [2 1 1] ;
Nt = 20 ;

%% example 2
clear
clc

f = @testFunction2 ;
p = 10 ;
x0 = [-9 -8 -7 -6] ;
p0 = 11 ;
T = 5 ;
Nt = [30 30 30 30] ;

%% example 3
clear
clc

f = @testFunction3 ;
p = 0.5 ;
x0 = [5 4] ;
p0 = 0.7 ;
T = [3 5] ;
Nt = [50 50] ;

%% example 4
clear
clc

f = @testDubins ;
p = [.9 2.6]' ;
x0 = [0 0 0]' ;
p0 = [1.5 1.5]' ;

T = 5 ;
Nt = 100 ;

%% Run this after running one of the example cells
ex.f = f ;
ex.p = p ;
ex.x0 = x0 ;
ex.p0 = p0 ;
ex.T = T ;
ex.Nt = Nt ;
ex.test_gradient = 1 ;
ex.run_solver = 1 ;
ex.plot_results = 0 ;
% ex.pl = [1 1];
% ex.pu = [1 2.5];

[sol,user] = test_example(ex) ;