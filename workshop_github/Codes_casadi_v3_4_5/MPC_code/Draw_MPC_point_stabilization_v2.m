function Draw_MPC_point_stabilization_v2(t,xx,xx1,u_cl,xs,N,rob_diam)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];
picSize = 0.5*[1 1]; 



r = rob_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    
    cla;
    
    % triangle parameters
    h_t = 0.14; 
    w_t = 0.09; 
    
    % plot reference state
    x1  = xs(1);
    y1  = xs(2); 
    th1 = xs(3);
    
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); 
    hold on;
    
    % Plot actual trajectory
    x1  = xx(1,k,1); 
    y1  = xx(2,k,1); 
    th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory    
    
    % plot robot circle
    plot(x1+xp,y1+yp,'--r'); 
    
    %% Load image 
    if k == 1
        % Read the image
        [imgOrig, map, alphaOrig] = imread('car.jpg');

        % Pad image with zeros to make it square
        sizImg = size(imgOrig);
        maxSiz = max(sizImg(1:2));
        [minSiz, minIdx] = min(sizImg(1:2));
        padSiz = round( (maxSiz - minSiz)/2 );
        padVec = [0 0 0];
        padVec(minIdx) = padSiz;
        img   = padarray(imgOrig,padVec,'both');
        alpha = padarray(alphaOrig,padVec,'both');

        % GET handle to current axes and move the plot axes to the bottom
        ha = gca;
        uistack(ha,'bottom');

        % Creating a new axes for the logo on the current axes
        ha2 = axes('position',[0 0 0.1 0.1]);

        % Adding a LOGO to the new axes
        im = image(ha2, img);
        set(im,'AlphaData', alpha);
        % colormap (map)  % Setting the colormap to the colormap of the imported logo image

        % Turn the handlevisibility off so that we don't inadvertently plot
        % into the axes again. Also, make the axes invisible
        set(ha2, 'handlevisibility','off','visible','off')
    end

    %%
    % Desired picture
    picCenter = [x1 y1];
    % picSize   = [1 1];
    picAngle  = rad2deg(th1);   % Bring angle to degrees

    % Adjust picture for plotting        
    picCorner = picCenter - picSize./2;    

    % Main axis properties
    axXLim   = ha.XLim;
    axYLim   = ha.YLim;
    axCorner = [axXLim(1),  axYLim(1)];
    axSize   = [diff(axXLim), diff(axYLim)];

    % Figure properties
    haPos = get(ha,'position');
    figCorner = haPos(1:2);
    figSize   = haPos(3:4);

    % Find desired second axis location
    ax2Corner = figCorner + (picCorner-axCorner) .* (figSize ./ axSize);
    ax2Size   = picSize .* (figSize ./ axSize);

    set(ha2,'position',[ax2Corner,ax2Size]); % Set corner and size of second axis
    set(ha2, 'View', [-picAngle, 90]);       % Rotate second axis
    
    % plot predicted trajectory
    if k < size(xx,2) 
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end          
    
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 1.8 -0.2 1.8])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
close(gcf)
%viobj = close(aviobj)
%video = VideoWriter('exp.avi','Uncompressed AVI');

% video = VideoWriter('exp.avi','Motion JPEG AVI');
% video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
% open(video)
% writeVideo(video,F)
% close (video)

figure
subplot(211)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -0.35 0.75])
ylabel('v (rad/s)')
grid on
subplot(212)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -0.85 0.85])
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on
