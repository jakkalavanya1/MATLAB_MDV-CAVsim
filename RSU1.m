function mean=RSU1(sigma,vavg_sr,cz_length,N,R2_xo)
%     input values are sigma(standard deviation),
%     vavg_sr = velocity measured by RSU1,
%     cz_length= length of control zone,
%     N=number of values to generate,R2_xo= initial positon of car
%     R2_ini(5,:)=contains the value of vav_sr
mu=vavg_sr;
%mu=13;
v=normrnd(mu,sigma,1,N); %now we get a vector of velocities
%v=[25:0.1:35] %velocity of vehicle lies between 25 to 35 
tm1=(cz_length-R2_xo(1,1)./v);
mu1=sum(tm1)/N; %for now use mu1 value, time at which vehicle enter merging zone
tm2=(cz_length-R2_xo(1,2)./v);
mu2=sum(tm2)/N;
mean=[mu1 mu2]
% tmax=mu1+sigma1
% tmin=mu1-sigma1 % the difference is very low
end
%F:\UNIVERSITY OF DELAWARE\lab\MATLAB Simulation
