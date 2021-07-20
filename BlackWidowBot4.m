%%                                                                 %%
% Black Widow Optimization  (BWO) source codes                      %
%                                                                   %
%  Developed in MATLAB R2020a                                       %
%                                                                   %
%  Author and programmer: Ayush Saun                                %
%                                                                   %
%% problem parameters required
clear;
clc;

Q1 = [-360 360]; % for finding Q1
Q2 = [-360 360]; % for finding Q2
Q3 = [-360 360]; % for finding Q3
Q4 = [-360 360]; % for finding Q4
Q5 = [-360 360]; % for finding Q5
Q6 = [-360 360]; % for finding Q6

Dim=2; % problem Dimention

% Define target for the end effector to achieve: xd
xd = [0.6 0.1 0.1];

RepNo=[1 , 1 , 1];
FITNESS = [0.1 0.1 0.1];
z=1;

l(1) = Link([0,0,0,pi/2,0]);
l(2) = Link([0,0,0.4318,0,0]);
l(3) = Link([0,0.15,0.0203,-pi/2,0]);
l(4) = Link([0,0.431,0,pi/2,0]);
l(5) = Link([0,0,0,-pi/2,0]);
l(6) = Link([0,0,0,0,0]);

puma=SerialLink(l)
puma.name= 'Puma Robot'

tic
bestvalue = bwoa(RepNo,Q1,Q2,Q3,Q4,Q5,Q6,Dim,xd,puma,FITNESS,z);
T = toc;

puma.fkine(bestvalue)

j=jacobe(puma,bestvalue);
q = (bestvalue - [0 0 0 0 0 0])/T;
x = j*q';

q0 = [0 0 0 0 0 0];
qa = (q - q0)/T;
xa = j*qa';

% TIME = timeit(@()bwoa(RepNo,Q1,Q2,Q3,Q4,Q5,Q6,Dim,xd,robot,FITNESS,z))
% bestvalue = bwoa(RepNo,Dim,xd,robot,FITNESS,z);