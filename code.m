function  code(sp)  
addpath('.');
addpath('./matlab_pioneer');

global  SensorsArray PureSensorsArray xOdometryOffset yOdometryOffset thetaOdometryOffset CurrentCorridor robot AfterHalfHall fileID Warray ;
fileID = fopen('log.txt','w');
AfterHalfHall = false;
Warray = [];
robot = sp;
SensorsArray = [];
PureSensorsArray = [];
xOdometryOffset = 0;
yOdometryOffset = 0;
thetaOdometryOffset = 0;
CurrentCorridor=0;
clf;
pause on;
plotMap
[yy,xx] = GetPoints();
x=3; xInicial = x;
y=23.5; yInicial = y;
theta=-pi/2; GInicial=theta;
diff=0;

for k=1:70000

        [l,e,yy,xx,diff] = getNexPoiny(x,y,theta,xx,yy,diff);


        [w,v] = CalculateMove(x,y,theta,l,e);
        
        prints("w: " +w+" v: " +v + " k " +k);
        MoveRobot(w,v);
        pause(0.1);
        UpdateSonars();
        [x,y,theta] = GetCurrentPosition(xInicial,yInicial,GInicial);


        figure(1)
        plot(x,y,'.'); 


        if(x <= 2.5 && y <= 25 && y > 23 && AfterHalfHall)
           prints("finish");
           MoveRobot(0,0);
           break; 
        end

end

MoveRobot(0,0);
figure(2)
plot(Warray,'-')


end

function   [w,v] = CalculateMove(x,y,theta,l,e)
   
    k1 = 0.6;    
    k2 = 0.6;
    

    prints("l= " +l+"e= "+e);
    v=200;
    w=k1*l + k2*e;

end

function  [l,e,yy,xx,diff] = getNexPoiny(xNow,yNow,thetaNow,xx,yy,diff)
    global CurrentCorridor;
    treshold=150;
    prints("CurrentCorridor: " +CurrentCorridor);

    l=[];
    counter = 0;
    for i=1:length(xx)-1

        counter = counter+1;
        if(counter>treshold)
            break
        end

        X = [xx(i),yy(i);xNow,yNow];
        l = [l;pdist(X,'euclidean')];

    end


    [l,ind] =min(l);
    

    y1=yy(ind);
    x1=xx(ind);

    y2=yy(ind+1);
    x2=xx(ind+1);

    %cut old points
    yy=yy(ind:length(yy));
    xx=xx(ind:length(xx));

    a=[x1-xNow;y1-yNow;0];
    b=[x2-x1;y2-y1;0];
    ab = cross(a,b);

    l=-l*sign(ab(3));

    thetaref = atan2(y2-y1,x2-x1);


    prints("pont = x1: " +x1+" y1: " +y1);
    thetaref=thetaref+diff;

    e = atan2(sin(thetaref-thetaNow), cos(thetaref-thetaNow));


    
    CurrentCorridor = GetCurrentCorridor(y1,x1);

end



function MoveRobot(w,v)
    global robot Warray;


   w = w*( 180/pi );

    if w <-0.001 && w > -1 
        w=-1;
    elseif w < 1 && w > 0.001
        w=1;
    else
        w=round(w);
    end
    Warray = [Warray w];
   prints("controls v="+v+" w="+w);
   pioneer_set_controls(robot,v,w);


end

function [x,y,theta] = GetOdometry()
   Pos = pioneer_read_odometry();

   x=Pos(1)/1000;
   y=Pos(2)/1000;

   %from 4096 metric to radians
   theta= (Pos(3))*( (2*pi)/4096 );

   if(Pos(3)<0)
    prints("problem......Simluated Robor, has negative angle");
   end


end




function [Sx,Sy,Stheta] = GetSensorsPosition(x,y,theta)
   global CurrentCorridor;
   
    LeftDistance = GetLeftDistance();
    prints("Sensor: "+LeftDistance);
    [x,y] = CheckFrontWall(x,y);
    
    %it a bad reading,we use only odometry
    if(LeftDistance <0)
            Sx = x;
            Sy = y;        
            Stheta=theta;
            return;
    end
    
    

    %Formula:  Coordenate = HallCenter + (0.75 - LeftDistance); for
    %corredor 1 and 2
    halfHall = 0.6;
   switch CurrentCorridor
        case 1
             Sx = x;
             Sy = 21 + (halfHall - LeftDistance);
        case 2
             Sx =  21 + (halfHall  - LeftDistance);
             Sy = y;
        case 3           
             Sx = x;
             Sy = 7.5 + (LeftDistance + 0.2 - halfHall);
        case 4
             Sx = 7.3 + (LeftDistance - halfHall);
             Sy = y;
       case 5 
             Sx = x;
             Sy = 21.3 + (LeftDistance  - halfHall);
       otherwise
            Sx = x;
            Sy = y;

   end


    

Stheta=theta;
end

function LeftDistance = GetLeftDistance()
   global SensorsArray PureSensorsArray;
   
   So =  sum(SensorsArray)/(3);
   SoPure = sum(PureSensorsArray)/(3);

   prints("L: "+ So(1)+ " R: " + So(8) );
   StopTest(SoPure);
   
   HallWidth=1.67;
    %its a bad reading lets read right side
    if(SoPure(1)> HallWidth)
        if(SoPure(8)> (HallWidth-0.380))
           So(1) = ReadOtherSensors();
           prints("Left and Right sensor are bad-");
        
        else
            %change/fix the left sensor 
            So(1) = HallWidth - So(8) - 0.380;
        end
    end
            

    LeftDistance = So(1);
    
    

end
function LeftDistance = ReadOtherSensors()
   global SensorsArray PureSensorsArray;
   
   So =  sum(SensorsArray)/(3);
   SoPure = sum(PureSensorsArray)/(3);
   
   
   HallWidth=1.6;
    if(SoPure(2) <= SoPure(7))
        if(SoPure(2)>HallWidth)
           LeftDistance = -1;
           prints("all sensors are bad");
           return; 
        end
        LeftDistance = So(2);             
    else
        if(SoPure(8)> (HallWidth-380))
           LeftDistance = -1;
           prints("all sensors are bad");
           return; 
        end
        LeftDistance = HallWidth - So(7) - 0.380;        
    end

    

end
function FrontDistance = GetFrontDistance()
   global SensorsArray;
   
   So =  sum(SensorsArray)/(3);
       
   prints("front sonars: " + So(4) + " " + So(5));
      
    FrontDistance = min(So(4), So(5));

end
function UpdateSonars()
   global SensorsArray PureSensorsArray;
   
   So = pioneer_read_sonars();
   
   So = So(1:8)/1000;
       
   prints("Sonars: " +So(1)+" "+So(2)+" "+" "+So(3)+" "+So(4)+" "+So(5)+" "+So(6)+" "+So(7)+" "+So(8));
   SensorsSize = size(SensorsArray);
    if(SensorsSize(1)<3)
        SensorsArray = [So ; SensorsArray];
        SensorsArray = [So ; SensorsArray];
        SensorsArray = [So ; SensorsArray];
        PureSensorsArray =SensorsArray;
    end
       

         
   PureSensorsArray = [So ; PureSensorsArray];

   PureSensorsArray(4,:) =0;
   PureSensorsArray(4,:) = [];
   
   S = sum(SensorsArray)/(3);
    for i = 1:8
        if(So(i)>4 && i ~= 4 && i ~= 5)
            So(i) = S(i);
        end
    end
   
   
   SensorsArray = [So ; SensorsArray];

   SensorsArray(4,:) =0;
   SensorsArray(4,:) = [];


end

function [x,y,theta] = GetCurrentPosition(xInicial,yInicial,GInicial)
    global xOdometryOffset yOdometryOffset thetaOdometryOffset;

    [xO,yO,thetaO] = GetOdometry();

    x=xO+xInicial;
    y=yO+yInicial;
    theta = thetaO + GInicial;
    [x,y] =  rotate_point(xInicial,yInicial,GInicial,x,y);

    x= x + xOdometryOffset;
    y= y + yOdometryOffset;
    thetaOdometryOffset = FixTheta(x,y,theta);
    theta= theta + thetaOdometryOffset;

    [Sx,Sy,Stheta] = GetSensorsPosition(x,y,theta);
    prints("Odometry= x: " +x +" y: " +y +" theta:  " + theta );

    
    
    xOdometryOffset = xOdometryOffset + Sx - x;
    yOdometryOffset = yOdometryOffset + Sy - y ;    

    x=Sx ;
    y=Sy ;

    prints("GetSensorsPosition= x: " +Sx +" y: " +Sy +" theta:  " + Stheta );
    prints("xOdometryOffset= "+xOdometryOffset+" yOdometryOffset= "+yOdometryOffset+" thetaOdometryOffset= "+thetaOdometryOffset);

end
function Ftheta = FixTheta(x,y,theta)
global CurrentCorridor thetaOdometryOffset FixDone;

Ftheta = thetaOdometryOffset;

   switch CurrentCorridor
        case 1
            if(x> 14 && x<15 && not(FixDone))
                prints("fixing");
                Ftheta = 0 - theta;
                FixDone =true;
            end
        case 2
            if(y> 14 && y<15 && not(FixDone))
                prints("fixing");
                Ftheta = -pi/2 - theta;
                FixDone =true;
            end
        case 3           
            if(x> 14 && x<15 && not(FixDone))
                prints("fixing");
                Ftheta = -pi - theta;
                FixDone =true;
            end
        case 4
            if(y> 14 && y<15 && not(FixDone))
                prints("fixing");
                Ftheta = -pi - pi/2 - theta;
                FixDone =true;
            end
       otherwise
            Ftheta=thetaOdometryOffset;
   end

end

function [x,y] = rotate_point(cx,cy,angle,px,py)
  s = sin(angle);
  c = cos(angle);

  %translate point back to origin:
  px = px-cx;
  py = py-cy;

  %rotate point
  xnew = px * c - py * s;
  ynew = px * s + py * c;

  %translate point back:
  x = xnew + cx;
  y = ynew + cy;

end
function [yy,xx] = GetPoints
yy=[];
xx=[];


    x = [3   3.1  3.2  3.6   4.3     15  ];
    y = [25  23   22 21.3  21    21  ];        
    xx = 3:.0005:15;

    yy = pchip(x,y,xx);
      
    plot(x,y,'o',xx,yy)
    
    x = [15 20   20.8   20.9  21   ];
    y = [21 21   20.8   20.5   13  ];       
    xxTmp = 15:.0005:21;

    yy = [yy pchip(x,y,xxTmp)];    
    xx = [xx xxTmp];
        
    plot(x,y,'o',xx,yy)

    BelowOffset = -0.0;
    
    x = [15              20              20.8               21 ];
    y = [7.5+BelowOffset 7.5+BelowOffset 8.5+BelowOffset    13]; 
    xxTmp = 15:.0005:21;

    yy = [yy fliplr(pchip(x,y,xxTmp))];    
    xx = [xx fliplr(xxTmp)];
        
    plot(x,y,'o',xx,yy)
    
    
    LeftOffset =  0.4;
    x = [6.8+LeftOffset  6.9+LeftOffset  7.1+LeftOffset   7.6+LeftOffset   15 ];
    y = [15              8.5+BelowOffset 8+BelowOffset  7.5+BelowOffset  7.5+BelowOffset]; 
    xxTmp = 6.8+LeftOffset:.0005:15;

    yy = [yy fliplr(pchip(x,y,xxTmp))];    
    xx = [xx fliplr(xxTmp)];
        
    plot(x,y,'o',xx,yy)
    
    EndOffset = 0.1;
    
    x = [2.3 2.80   4.3    6     6.5+LeftOffset   6.79+LeftOffset  6.8+LeftOffset ];
    y = [26  22.2+EndOffset   21.3+EndOffset   21.3+EndOffset  21.2+EndOffset   21+EndOffset   15]; 
    xxTmp = 2.3:.0005:6.8+LeftOffset;

    yy = [yy fliplr(pchip(x,y,xxTmp))];    
    xx = [xx fliplr(xxTmp)];
        
    plot(x,y,'o',xx,yy)


end
function plotMap
x1=0;
x2=28.5;
y1=0;
y2=28.5;
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
plot(x, y, 'b-', 'LineWidth', 3);
hold on;

x1=0;
x2=8.25;
y1=20.25;
y2=28.5;
x = [x1, 6.75, 6.75,   x2, x2, x1, x1];
y = [y1, y1,   y1+1.5, y1+1.5, y2, y2, y1];
plot(x, y, 'b-');

x1=8.25;
x2=20.25;
y1=8.25;
y2=20.25;
x = [x1, x2, x2, x1, x1];
y = [y1, y1, y2, y2, y1];
plot(x, y, 'b-');


x = [6.75,  6.75, 21.75, 21.75, 8.25];
y = [20.25, 6.75, 6.75,  21.75, 21.75 ];
plot(x, y, 'b-');


%Current to the door
x = [4.5  , 6.75];
y = [21.25, 21.25];
plot(x, y, 'b-');


xlim([-1, 30]);
ylim([-1, 30]);

end
function CurrentCorridor = GetCurrentCorridor(y,x)
global AfterHalfHall FixDone;

        if( x>5 && x<20 && y>19 && y<24 && not(AfterHalfHall))
            CurrentCorridor=1;
        elseif( x>20 && x<24 && y>8.5 && y<20)
            AfterHalfHall=true;
            CurrentCorridor=2;
        elseif( x>9 && x<20 && y>6 && y<8)
            CurrentCorridor=3;
        elseif( x>6 && x<9 && y>8.2 && y<20)
            CurrentCorridor=4;
        elseif( x>1 && x<6.6 && y>21 && y<21.75 && AfterHalfHall)
            CurrentCorridor=5;
        else
            FixDone=false;
            CurrentCorridor=0;
        end



end
function [x,y] = CheckFrontWall(x,y)
global   CurrentCorridor;

   FrontDistance = GetFrontDistance();
   %ignore more than 3 meters
   if(FrontDistance > 2)
      return;
   end


   
   if(CurrentCorridor == 1 && x > 19)
        prints("FrontDistance: "+FrontDistance);
        x = 21.75 - FrontDistance;
   elseif(CurrentCorridor == 2 && y < 9.75)
        prints("FrontDistance: "+FrontDistance);
       % y = 6.75  + FrontDistance +0.3;
   elseif(CurrentCorridor == 3 && x < 9.75)
        prints("FrontDistance: "+FrontDistance);
        x = 6.75  + FrontDistance;
   elseif(CurrentCorridor == 4 && y > 18.75)   
        prints("FrontDistance: "+FrontDistance);
        y = 21.75 - FrontDistance;       
   end

end
function StopTest(ArrayAverage)

    if(ArrayAverage(4) < 0.3 || ArrayAverage(5) <0.3)
    prompt = 'Obstacle, press key to continue? ';    
    MoveRobot(0,0);
    input(prompt,'s');
    
    end

end
function prints(s)
global fileID;

disp(s);
fprintf(fileID,s+"\n");

end

