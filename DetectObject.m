function state = DetectObject(cam)
%cam = cameraboard(mypi,'Resolution','160x120');
%resolution is set low to set speed
counter = 0;
state = 0;
for ii = 1:1
    img = snapshot(cam);
    %     %We process only every 4th frame to increase speed
    %     if mod(ii,4) == 0
    for xx = 1:120
        for jj = 1:160
            
            if (img(xx,jj,1) > 150 && img(xx,jj,1) < 170) && (img(xx,jj,2) > 220 && img(xx,jj,2) < 290) && (img(xx,jj,3) < 80)
                counter = counter + 1; %used to check that an image does in fact have the right color
                
                if counter >=20
                    state = 1;
                    counter = 0; %reset counter
                end
                
            end
        end
    end
end
end




