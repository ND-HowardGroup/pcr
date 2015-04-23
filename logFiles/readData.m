clc
clear

fg = clf;
ax = cla;

while(1)
    % system('tail -n +3 log.txt > tmp.txt');
    system('tail -n +3 log2__22Apr2015.txt > tmp.txt');
    filename = 'tmp.txt';
    A = importdata(filename);
    A(end,:)'

    t = A(:,1)./60*60;
    timeRem = A(:,2)./60;
    c = A(:,3);
    Tsp = A(:,4);
    T1 = A(:,5);
    T2 = A(:,6);
    Tavg = A(:,7);
    heater = A(:,8);
    fan = A(:,9);
    stage = A(:,10);


    fan = 40*fan;

    figure(fg);
    axes(ax);
    
    plot(t,Tavg, t,Tsp);
%     plot(t,Tavg, t,Tsp, t,heater/255*40, t,fan,'k.');
%     plot(t,T1, t,Tavg, t,T2);

%     plot(   t,T1,'-k',  ...
%             t,T2,'-r',  ...
%             t,Tavg,'-b',  ...
%             t,Tsp,'-m',  ...
%             t,heater/255*40,'-r',  ...
%             t,fan,'k.'  ...
%         );

    ylim([0 110]);
    xlabel('Time (sec)');
    ylabel('Temperature (^{\circ}C)');
    set(gca,'XMinorTick','on');
    grid on;
%     legend({'T_{avg}', 'T_{sp}', 'Heater%'}, 'location','southwest')

%     break;
%     legend({'T_{block}', 'T_{avg}', 'T_{tube}'}, 'location','northwest')
    drawnow;
    pause(1);
    
end
