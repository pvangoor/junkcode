%% Data import
clear all
close all

goodDataStart = 600;
% goodDataEnd = 2.2*1e5;
goodDataEnd = 2.2e5;
raw_data = dlmread('phone_stationary3.csv', ',', [goodDataStart,0,goodDataEnd,12]);
% raw_data = dlmread('phone_stationary3.csv', ',', goodDataStart,0);

timestamp = raw_data(:,1);
% gyro = raw_data(:,2:4);
gyro = raw_data(:,5:7);
acc = raw_data(:,11:13);



%% Data process

tau0 = mean(diff(timestamp));
sampleNum = 20;

tau = sampleNum*tau0;

angles = cumsum(diff(timestamp).*gyro(2:end,:),1);


sampleSelection = 10:100:20000;
variances = NaN(size(sampleSelection,1),3);
variances2 = NaN(size(sampleSelection,1),3);
for s = 1:numel(sampleSelection)
%     variances(s,:) = computeGyroVar(gyro, tau0, sampleSelection(s));
    variances2(s,:) = computeGyroVar2(acc, tau0, sampleSelection(s));
    
end
taus = sampleSelection * tau0;
loglog(taus,variances2,'-o');
grid;

function gyroVar = computeGyroVar(angles, tau0, m)
N = size(angles,1);
tau = tau0*m;
gyroVar = 1/(2*tau^2 * (N-2*m)) * sum( (angles(1+2*m:N,:) - 2*angles(1+m:N-m,:) + angles(1:N-2*m,:)).^2 ,1);
end

function gyroVar = computeGyroVar2(gyro, tau0, m)
N = size(gyro,1);
tau = tau0*m;
gyroVar = 1/(2*tau^2 * (N-m-1)) * sum( (gyro(1+m:N,:) - gyro(1:N-m,:)).^2 ,1);
end


function allanVar = computeAllanVar(data, sampleNum)
n = size(data,1);
averages = NaN(n-sampleNum, 3);
for i = 1:(n-sampleNum)
    averages(i,:) = mean(data(i:i+sampleNum-1,:),1);
end

allanVar = 1/(2*(n-sampleNum-1)) * sum(diff(averages,1),1);

end