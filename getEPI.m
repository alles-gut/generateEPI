%get EPI from light field images
%with horizontal manner

function epi_img = getEPI(filepath, height, epline, size, start)
   %height = height of EPI
   %epline = position of EPI slice
   %size = image resolution x
   %start = number of image start to stack
   
   data_type = 'png';
   flist = dir(fullfile(filepath,['*.' data_type]));
   
   epi_img = zeros(height, size, 3);
   for i = start:start + height - 1
       img = imread(fullfile(flist(i).folder,flist(i).name));
       epi_img(i-start+1,:,1) = cast(img(epline,:,1), 'double')/255;
       epi_img(i-start+1,:,2) = cast(img(epline,:,2), 'double')/255;
       epi_img(i-start+1,:,3) = cast(img(epline,:,3), 'double')/255;
   end
       
end
