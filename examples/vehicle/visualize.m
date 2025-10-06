function visualize(sticky)

    close all;
    
    x_eta = 0.2;
    y_eta = 0.2;
   
    
    if(nargin == 0)
        sticky = true;
    end

    % initial vals
    colors = get(groot,'DefaultAxesColorOrder');
    figure('pos',[10 10 900 700])
    axis([-0.5 10.5 -0.5 10.5]);
    hold on;

    frame_count = 1;
    F(frame_count) = getframe(gcf);
    frame_count = frame_count+1;

    count = 1;
    objs = [];
    pwin = [];
    pc2pre = [];
    while true
        wb_file = ['vehicle_workboxes_' num2str(count) '.txt'];
        if exist(wb_file, 'file') == 0
            if sticky
                continue;
            else
                break;
            end
        end

        if(~isempty(objs))
            undrawWorkBoxs(objs);
        end
        if(~isempty(pwin))
            delete(pwin);
        end
        if(~isempty(pc2pre))
            delete(pc2pre);
        end        

        % read files
        [x, y, w, h, v] = getWorkBoxs(wb_file);
        
        
        if(w <= 0 || h <= 0)
            return;
        end
        
        % print work-boxes
        objs = drawWorkBoxs(x, y, w, h);
        drawnow;
        F(frame_count) = getframe(gcf);
        frame_count = frame_count+1;

        count = count+1;        
    end

    % create the video writer
    writerObj = VideoWriter('robot.avi');
    writerObj.FrameRate = 4;
    writerObj.Quality = 100;
    
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i=1:length(F)
        % convert the image to a frame
        frame = F(i) ;    
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
    
    
    function [x, y, w, h, v] = getWorkBoxs(wb_file)
        fid = fopen(wb_file);
        line_count = 1;
        while true
            line = fgetl(fid);
            if line ~= -1
                splitted = strsplit(line);
                x(line_count) = x_eta*str2double(splitted{1});
                y(line_count) = y_eta*str2double(splitted{2});
                w(line_count) = x_eta*str2double(splitted{3});
                h(line_count) = y_eta*str2double(splitted{4});
                v(line_count) = str2double(splitted{3})*str2double(splitted{4});
            else
                break;
            end
            line_count = line_count + 1;
        end
        fclose(fid);
    end
    function objs = drawWorkBoxs(x, y, w, h)
        e=0.05;
        for i = 1:length(x)
            objs(i) = rectangle('Position',[x(i)-e, y(i)-e, w(i)+e/2, h(i)]+e/2, 'EdgeColor', 'k', 'LineWidth',3);
        end
    end
    function undrawWorkBoxs(objs)
        for i = 1:length(objs)
            delete(objs(i));
        end
    end
end

