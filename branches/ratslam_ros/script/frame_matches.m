% frame_matches.m
% 
% .m file to load reported frame matches and visualize the two frames
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

clc;
clear all;
close all;

m1 = csvread('vt_id.dat', 1, 0);
ml = length(m1);
start = m1(1, 1) / 1e9;
t1 = m1(1:ml, 1) / 1e9 - start;

frame_offset = 0;
match_buffer = 10;   % This is the minimum "gap" between the current frame and the matched frame that we will generate image output for.

v1 = m1(1:ml, 5);
v1 = v1 + 1;

vn = 0;
vframes = zeros(length(v1), 1) - 1;

pair_match = [];

for c = 1:length(v1)
    if v1(c) > vn
       vn = v1(c);
       vframes(v1(c)) = m1(c, 2) + 1;
    elseif v1(c) < vn - match_buffer
        pair_match = [pair_match; c vframes(v1(c))];
        
    end
end

pair_match = pair_match + frame_offset;

% mov = mmreader('E:\datasets\car_driving\stlucia_large\stluciahq.avi');
% mov = mmreader('E:\datasets\iRat\aus_map\log_irat_red.avi');
mov = mmreader('newcollege_pano.avi');
num_frames = get(mov, 'numberOfFrames');
fh = get(mov, 'Height');
fw = get(mov, 'Width');
disp(['Num frames: ' int2str(num_frames) '. (width, height): ' int2str([fw fh])])

npmatches = size(pair_match);
npmatches = npmatches(1);

length(pair_match)

for i = 1:length(pair_match)
    im1 = [];
    im2 = [];
    
    %     Horizontal arrangement
    buf_width_vert = 20;
    buf_width_horz = buf_width_vert;
    
    
    vid_frame1 = read(mov, pair_match(i, 1));
    vid_frame2 = read(mov, pair_match(i, 2));
    tsz = size(vid_frame1);
    vert_buffer = zeros(tsz(1), buf_width_vert, 3) + 255;
    
    im_comb = [vid_frame1 vert_buffer vid_frame2];
    
    imagesc(im_comb);
    axis off;
    
    axis tight;
    
    set(gca,'LooseInset', [0 0 0 0])
    set(gca,'Position', [0 0 1 1]);
    
    
    im_comb_sz = size(im_comb);
    ratio = im_comb_sz(2) / im_comb_sz(1);
    psize = 2;
    set(gcf, 'PaperPosition', [0  0.0  psize * ratio   psize]);
    
    drawnow;
    pause(0.01);
    
    
    framename = sprintf('frame_%.6d.jpg', i);
    
    print('-r200', '-djpeg100', framename);
    
end