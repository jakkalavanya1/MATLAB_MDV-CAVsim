% model values are the actual values with which cars travel

close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
clc;    % Clear the command window.

v1 = 13.5; % iniial velocity [m/s]
a_model = -0.1; %constant deceleration [m/s^3]
% to make sure that same random variable is generated each time
s1 = 500; %total length of secondary road (SR)
s2 = 400; % from RSU2 till entry of merging zone 
s3 = 100; % distance between two RSU
s = rng;
vf = 12 + (14-12).*rand(1); 
rng(s);
vf_model = 12 + (14-12).*rand(1); 
%==============simulating for entire time CREATEING MODEL============
a_model=-0.1;
vf_model = 12 + (14-12).*rand(1);
s2(1)=1;
v2_model(1) = sqrt((vf_model)^2-2*a_model*s2(1));
t=100; %seconds
dx=500/t;
for i=2:t
    s2(i)=s2(i-1)+dx;
    v2_model(i) = sqrt((vf_model)^2-2*a_model*s2(i));
    if s2(i)==100
        v2_RSU2=v2_model(i);
    end
end
%============================ PREDICTION ==============================

% Generate values from the uniform distribution on the interval [a, b].
% r = a + (b-a).*rand(100,1)
v2_model = sqrt((vf_model)^2-2*a_model*s2);

% predict final velocity and v2 using Gaussian distribution
vf=normrnd(13.5,1) %try to make it skewd to left
a=(vf^2-v1^2)/(2*s1)
v2=sqrt(v1^2+2*a*(s3)) % vel at RSU2 predicted using acclf from gaussain

%=====================================================

plot(s2,v2_model)
    % caluclating the error

error=v2-v2_RSU2;
if error >1
    vf_consider=vf %take final velociy predicted from gaussian
else
    vf_consider = vf_model %then the initial predicted can be there as
                            % it does not make much difference
end

% Least Squares Optimization
%   cost function is --velocity--
%   try to minimize the error in cost funcion
error = sqrt(sum((v2_model-v2)^2))

s2_vec=[100:500];
s3_vec=[1:100];
v1 = 13.5;
a=-0.1;
v2=sqrt(v1^2+2*a*(s3_vec)) %gaussian-the accln value came from gaussian
theta_flip=polyfit(s3_vec,v2,1)
theta=fliplr(theta_flip)
v2_update=theta(1)*


%=========2 VEHICLES, DISTANCE CONSTRAINT===========

s(1)=
for i=1:t %timr
    for j=1:2 %number of vehicles
        s(i)=s(i-1)+dx;
        t(i)=t(i-1)+dt;
        x(i,j)=-120+v2_model(1)*t(i)  %say car starts at 125 meters before control zone(500m)
        
        

