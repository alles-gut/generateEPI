function [R,G,B] = getColor(filepath)

center = 'input_Cam040.png';
img = imread(fullfile(filepath, center));

R = [];
G = [];
B = [];
for i = 1:size(img, 2)
    for j = 1:size(img, 1)
        R = [R, cast(img(j,i,1), 'double')/256];
        G = [G, cast(img(j,i,2), 'double')/256];
        B = [B, cast(img(j,i,3), 'double')/256];
    end
end

end