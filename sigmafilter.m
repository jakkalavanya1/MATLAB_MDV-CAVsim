function [filt_data,indices]=sigmafilter(data);
% [filt_data,indices]=sigmafilter(data);
% run a three sigma filter repeatedly until no more data lie outside the
% three sigma filter
%
%input: data is data time series to be filtered
%       
% output: filt_data is the vector of filtered data with the "outliers"
% removed
% indices are the indices in the original vector where data are thought to
% be "outliers".  


% change "outliers" to Nans first so we can retain indices, then remove
% those NaNs as a last step

flag =1; % set flag for if any data points still outside the three sigma window
indices=[]; % initiate indices
ct=0; %initiate counter

data=data(:); % verify data as a column vector

while flag == 1	% start while loop
    clear avg stand
	
    ct=ct+1 % increment counter
	avg=nanmean(data); % mean with NaNs removed
	stand=nanstd(data); % standard deviation with NaNs removed
    
    l=find(data > (avg+3*stand) | data < (avg-3*stand)); % find outliers
    
    if isempty(l)==0  % determine if any "outlier" data were found
    data(l)=NaN*l;  % reset those values to NaN
    indices=[indices l'];  % concatenate indices of outliers
    else
        flag=0; % set flag to indicate no more outliers and end the while loop
    end
end
    
sprintf('It took %d passes to remove "outliers"',ct)  % output information to he screen
filt_data=data; % set output data

