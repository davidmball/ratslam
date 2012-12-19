% showem.m
% 
% .m file to load and visualize experience map snapshots logged by OpenRatSLAM at regular
% intervals. Set index_file to 1 for the first run to parse the indices of
% the start of each line in the map.dat logfile, and then set it to 0 for subsequent use.
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


clear all;
close all;
clc;

draw_links = 1;
index_file = 0; % Set this to 1 to index the file and save fseek indices to a .mat file
single_mode = 1;
row = 30;

%fid = fopen('c:/ratslam experimental results/exp_map_movie_file.txt', 'r');
fid = fopen('map.dat', 'r');

f = 1;
font_size = 16;

num_loops = 0;
lp = 1;
fig_hand = figure;
drawnow;
pause(0.1);
tic

rota = 0;

markersize = 5;
linewidth = 2;
fps = 10;
% Set the gridspacing to a dimension suitable for the dataset i.e. 
% 500 for St Lucia dataset, 0.5 for the iRat dataset.
% gridspacing = 500;
% gridspacing = 0.5;  
gridspacing = 50;

mpart = 1;

fid = fopen('map.dat');

if index_file == 1
    row = 1;
    a = textscan(fid, '%s[^\n]', 1);
    fseeks = [];
    disp(['Parsing to snapshot #' num2str(row)]);
    for j = 1:1e6
        dummyline = fgetl(fid);
        if dummyline == -1
            break;
        end
        fseeks = [fseeks ftell(fid)];
        save fseeks.mat fseeks        
        j
    end
    disp('Finished parsing');
    return
end

load fseeks
if (row > length(fseeks))
    disp('Resetting row to end of file');
    row = length(fseeks) - 1;
end

disp(['Seeking to ' num2str(row)]);
fseek(fid, fseeks(row), 'bof');
disp('Seek done');



while( ~feof(fid) && num_loops < 100000)

    a = textscan(fid, '%n,%n,%n,,%n,', 1)
    a = [a{:}];
    
    init_offset = 4;
    
%     n = a{init_offset}   
    n = a(init_offset);
    
    elen = 8;
    
%     b = csvread('map.dat', row, 5, [row 5 row 5 + n * elen - 1]);
    disp('Loading nodes');
    b = textscan(fid, '%n,%n,%n,%n,%n,%n,%n,%n,', n);
    b = [b{:}];
    
    blen = size(b);
    blen = blen(1);
    
    pose = reshape(b', elen, blen)';
    
    disp('Loading links');
    nl = textscan(fid, '%n,', 1);
    nl = [nl{:}]
    
    llen = 11;
    links = textscan(fid, '%n,%n,%n,%n,%n,%n,%n,%n,%n,%n,%n,', nl);    
    links = [links{:}];
    
    dummy = textscan(fid, '\n', 1);
    
    disp('Visualizing nodes');
    
    plot(pose(:, 2), pose(:, 3), 'go', 'MarkerSize', markersize, 'MarkerFaceColor', 'none', 'LineWidth', linewidth)
    axis equal;
    
    if draw_links == 1
        disp('Visualizing links');
        
        if nl > 0
            
            for k = 1:nl
                sn = links(k, 2) + 1;
                dn = links(k, 3) + 1;
                line([pose(sn, 2) pose(dn, 2)], [pose(sn, 3) pose(dn, 3)]);                
            end
        end
    end
       
    
    xlabel('x (m)');
    ylabel('y (m)');
    
    set(gca,'LineWidth',2,'FontSize',font_size);
    set(get(gca, 'Xlabel'), 'FontSize', font_size);
    set(get(gca, 'Ylabel'), 'FontSize', font_size);
          
    set (gca, 'Xtick', [-3000:gridspacing:3000])
    set (gca, 'Ytick', [-3000:gridspacing:3000])    
    grid on;    
    axis equal;
    axis tight;
    taxis = axis;
    taxis = taxis + 0.25 * [-gridspacing gridspacing -gridspacing gridspacing];
    axis(taxis);
    drawnow;
    
    disp('Saving to file...');
    fname = sprintf('em_map_%4d', row);
    print('-r300', '-djpeg100', fname);    
    disp('done.');

    if single_mode == 1
        break;
    end

end

fclose(fid);
