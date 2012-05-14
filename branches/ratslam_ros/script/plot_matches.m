% plot_matches.m
%
% .m file to load and visualize visual templates as true positives, false
% positives etc...
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

close all;
clear all;
clc;

% Load ground truth files

load irat_gt2.mat;
% x, y, z, timestamp
gt = gt(:, [3:5 1]);
gt_offset = 6;
gt = [repmat(gt(1, :), [gt_offset 1]); gt];

lt = [];
n = 0;

figh = figure;

% set(figh, 'Position', [1          83        1680         898]);
set(figh, 'Position', [9          49        1350         642]);
% subplot(1, 2, 1);
plot(gt(:, 1), gt(:, 2), 'c', 'LineWidth', 0.25);
axis equal;
hold on;

% subplot(1, 2, 2);
% plot(gt(:, 1), gt(:, 2), 'k-', 'LineWidth', 0.5);
% axis equal;
% hold on;

c = 1;

% Read in the visual template ID and experience map ID data from the ROS
% bagfile

m1 = csvread('vt_id.dat', 1, 0);
ml = length(m1);
start = m1(1, 1) / 1e9;
t1 = m1(1:ml, 1) / 1e9 - start;

v1 = m1(1:ml, 5);
v1 = v1 + 1;
% ylabel('Template Number');

m2 = csvread('em_id.dat', 1, 0);
ml2 = length(m2);
t2 = m2(1:ml2, 1) / 1e9 - start;
v2 = m2(1:ml2, 6);
v2 = v2 + 1;

t1 = t1 * 2;
t2 = t2 * 2;

% Find out minimum maximum timestamp of both files
tmax = min(max(t1), max(t2));
tstep = 0.01;

trng = 0:tstep:tmax;

vt_c = 1;
em_c = 1;
gt_c = 1;

vn = 0;
en = 0;


ti = [];
ei = [];

error_thresh = 0.1;

c = 1;

fp = [];
nfp = 0;
tp = [];
ntp = 0;

for c = 1:length(t1)
    
    t = t1(c);
           
    while (gt(gt_c, 4) < t)
        gt_c = gt_c + 1;
    end
    
    %     Print current indices
    %     [vt_c em_c gt_c]
    
    if (v1(c) > vn)
        %         New template        
        vn = v1(c);
        ti(vn) = gt_c;
        gt_t(c, :) = gt(gt_c, :);
    else
        %        Recognition
      gt_t(c, :) = gt(ti(v1(c)), :);
    end
    
    
    
    if (hypot(gt_t(c, 1)-gt(gt_c, 1), gt_t(c, 2)-gt(gt_c, 2)) > error_thresh)
        plot([gt_t(c, 1) gt(gt_c, 1)], [gt_t(c, 2) gt(gt_c, 2)], 'k-', 'LineWidth', 2);
%         fp = [fp; 1];
        nfp = nfp + 1;
    else
%         plot([gt_t(c, 1) gt(gt_c, 1)], [gt_t(c, 2) gt(gt_c, 2)], 'g-', 'LineWidth', 2);
        ntp = ntp + 1;
    end
    
end

% figure;
% plot(v1(1:c), '.');

% subplot(1, 2, 1);
% plot(gt_t(:, 1), gt_t(:, 2), 'x');
axis equal;
axis tight;

print -r300 -dpng ground_truth.png

return;

cloud_rng = 0.2;

dthresh = zeros(1, size(l, 1)) + thresh;

for i = frng
    
    if length(lt) == 0
        lt = l(i, :);
        n = 1;
        bm(c) = n;
    else
        lt = [lt; l(i, :)];
        
        if d > thresh
            bm(c) = n;
        else
            % Match obtained, add a particle
            if (bm(c) < n - recent_buff)
                p = [p; gt(floor(bm(c) - 1 / 2) + 1, 1:3) 0 i bm(c)];
            end
            
        end
        
        
        
        n = n + 1;
        
        %             if d > thresh
        %                 lt = [lt; l(i, :)];
        %                 bm(c) = n;
        %                 n = n + 1;
        %             else
        %
        %             end
    end
    
    c = c + 1;
end


%%
pthresh = 2.0;
error_thresh = 0.25;
figure;
subplot(1, 2, 1);
hold on;
plot(gt(:, 1), gt(:, 2), 'b-', 'LineWidth', 0.1);
axis equal;
title(['Loop closure with IR, cyan correct, black lines error > ' num2str(error_thresh)]);

ntp = 0; nfp = 0;

plc = find(lc(:, 2) > pthresh);

fp = [];
f
for k = 1:length(plc)
    gt1 = gt(lc(plc(k), 4), :);
    gt2 = gt(lc(plc(k), 5), :);
    
    plot([gt1(1) gt2(1)], [gt1(2) gt2(2)], 'ro', 'MarkerSize', 10);
    if (hypot(gt1(1)-gt2(1), gt1(2)-gt2(2)) > error_thresh)
        plot([gt1(1) gt2(1)], [gt1(2) gt2(2)], 'k-', 'LineWidth', 2);
        fp = [fp; lc(plc(k), 4)];
        nfp = nfp + 1;
    else
        plot([gt1(1) gt2(1)], [gt1(2) gt2(2)], 'c-', 'LineWidth', 2);
        ntp = ntp + 1;
    end
    
end

subplot(1, 2, 2);

% % print -r300 -dpng ltemplates.png