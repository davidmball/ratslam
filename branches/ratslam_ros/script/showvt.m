% showvt.m
% 
% .m file to load and visualize visual template and experience map histories
% for a dataset, from logfiles created by OpenRatSLAM
% 
% Copyright (C) 2012
% Michael Milford (michael.milford@qut.edu.au)
% 
% 1. Queensland University of Technology, Australia
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.


% Plot a template graph

clear all;
close all;
clc;
figh = figure;

m1 = csvread('vt_id.dat', 1, 0);
ml = length(m1);
start = m1(1, 1) / 1e9;
t1 = m1(1:ml, 1) / 1e9 - start;

v = m1(1:ml, 5);
plot(t1, v, 'rx');
% ylabel('Template Number');

m2 = csvread('em_id.dat', 1, 0);
ml2 = length(m2);
t2 = m2(1:ml2, 1) / 1e9 - start;
v2 = m2(1:ml2, 6);

hold on;
plot(t2, v2, 'b.');
hold off;

xlabel('Time (s)');
ylabel('Template / Experience ID Number');
set(gca,'LineWidth',1,'FontSize',14);
set(get(gca, 'Xlabel'), 'FontSize', 14);
set(get(gca, 'Ylabel'), 'FontSize', 14);
% axis tight;

% figure;
% hist(v, [0:1:ceil(max(v))]);
axis tight;
legend('Template ID', 'Experience ID', 'Location', 'NorthWest');

print -r300 -djpeg100 vt_em_ids.jpg