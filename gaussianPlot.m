function mu1=gaussianPlot(sigma,vavg_sr,cz_length,N)
%     input values are sigma(standard deviation),
%     m= mean which is equal to v1,v1 = velocity measured by RSU1,
%     L= length of control zone,N=number of values to generate
sigma=5;%m/s
v1=13; %m/s
mu=v1;
cz_length=400; % 400m 
R2_xo=[-150 -125];
N=1000;
v=normrnd(mu,sigma,1,N); %now we get a vector of velocities
%v=[25:0.1:35]
%N=length(v);
%gives a time vector
tm=(cz_length-R2_xo(1,1))./v;
mu1=sum(tm)/N %for now use mu1 value
tm1=tm-mu1;
sigma1=sqrt((sum(tm1)^2)/N)
%where mu1 and sigma1 are the values from RSU1
%format long
tmax=mu1+sigma1
tmin=mu1-sigma1
%plot(tm,v,'*')
end
