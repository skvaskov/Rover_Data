function [ dzdt,dfdz,dfdp ] = wildshit( z,u,p )
%WILDSHIT Summary of this function goes here
%   Detailed explanation goes here

u=u(1);

dzdt=[p*(u)];

dfdz=[0];

dfdp=[u];


end

