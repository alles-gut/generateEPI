clc; clear; close all;

addpath('lf-tools');
trainpath = 'training';
labelpath = 'label';
mkdir(trainpath);
mkdir(labelpath);

dir_list  = ["training/boxes","training/cotton","training/dino","training/sideboard", ...
             "additional/boardgames","additional/kitchen","additional/medieval2", ...
             "additional/museum","additional/pens","additional/pillows", ...
             "additional/platonic","additional/rosemary","additional/table", ...
             "additional/tomb","additional/town","additional/vinyl", ...
             "additional/antinous","additional/dishes","additional/greek","additional/tower"];
dpath     =  'lf-tools';
epline    = [(5:95)*5];

for di = 2:length(dir_list)
    for ei = 1:length(epline)
        getEPIdataset(dir_list(di),  epline(ei));
    end
end