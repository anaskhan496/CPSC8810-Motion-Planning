raw_image = imread('world.png');
img = rgb2gray(raw_image)
img_new = zeros(size(img,1), size(img,2));
for i = 1: size(img,1)
    for j = 1:size(img,2)
        if (img(i,j) < 140 || img(i,j) > 220)
            for ii = -10:10
                for jj = -10:10
                    if i+ii>0 && j+jj>0
                        img_new(i+ii,j+jj)=1;
                    end
                end
            end
        end
    end
end
figure(1);
imshow(img_new)
imwrite(img_new,'point_map_10.png')