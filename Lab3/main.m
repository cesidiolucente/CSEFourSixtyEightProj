function main()

  image= "image1.jpg"; #calls merge image function
  mergeimage(image); #told to do this in main but I thought this would be more efficient 
  image= "image2.jpg";
  mergeimage(image);
  image= "image3.jpg";
  mergeimage(image);
  image= "image4.jpg";
  mergeimage(image);
  image= "image5.jpg";
  mergeimage(image);
  image= "image6.jpg";
  mergeimage(image);
  
  

  printf("SSD alignments \n" ); #calls then prints ssd funtion sending image string
  image= "image1.jpg";  
  [x,y]=im_align1(image);
  printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image2.jpg";
  [x,y]=im_align1(image);
  printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image3.jpg";
  [x,y]=im_align1(image);
  printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
   image= "image4.jpg";  
  [x,y]=im_align1(image);
  printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image5.jpg";
  [x,y]=im_align1(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image6.jpg";
  [x,y]=im_align1(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  
  
  printf("NCC alignments \n"); #calls then prints ncc 
  image= "image1.jpg";  
  [x,y]=im_align2(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image2.jpg";  
  [x,y]=im_align2(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image3.jpg";  
  [x,y]=im_align2(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image4.jpg";  
  [x,y]=im_align2(image);
  printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image5.jpg";  
  [x,y]=im_align2(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  image= "image6.jpg";  
  [x,y]=im_align2(image);
   printf(image);
  printf(" x:");
  printf("%d", x);
  printf(" y:");
  printf("%d", y);
  printf(" \n");
  printf("finished");
  
 endfunction
 
 function mergeimage(image) #simple alignment and merge
  info=imfinfo(image); #gets image info mainly height
 totheight=info.Height; #initially used to be modular, but created issues
 totwidth=info.Width;
 
 firstt=341; #manual data added for cropping purposes height of first pic
 secondt=682; # from first to second is height of second
 thirdt=1023;
 
 
 
 
 Blue = imread(image,"PixelRegion", {[0 firstt], [0 totwidth]}); #reads each color chanel seperately 
 Green = imread(image,"PixelRegion", {[firstt secondt], [0 totwidth]});
 Red = imread(image,"PixelRegion", {[secondt thirdt], [0 totwidth]});
 
 blueblue=Blue; #initially used to convert each image to its own color channel but not needed
 greengreen=Green;
 redred=Red;
 
 
 
 t=zeros(firstt,totwidth,3,"uint8"); # creates 3d matrix
 t(1:342,1:totwidth+1,1)=redred; #fills red matrix with the red channel
 t(1:342,1:totwidth+1,2)=greengreen; #fills green
 t(1:342,1:totwidth+1,3)=blueblue; #fills blue
 
 image=image(1:end-4); #removes the .jpg from image string 

 image = strcat(image, "-color.jpg"); #adds to end of image string to make call more modular and easier
 imwrite(t,image); #writes matrix to image
 
 
 
 endfunction
  
 function [x,y]=im_align1(image)
 info=imfinfo(image); #same purpose as in previous function, only width is used
 totheight=info.Height;
 totwidth=info.Width;
 
 firstt=341;
 secondt=682;
 thirdt=1023;
 
  Blue = imread(image,"PixelRegion", {[0 firstt], [0 totwidth]});
 Green = imread(image,"PixelRegion", {[firstt secondt], [0 totwidth]});
 Red = imread(image,"PixelRegion", {[secondt thirdt], [0 totwidth]});
 
 te=zeros(firstt,totwidth,1,"uint8"); #creates matrix for just blue image/channel
 te(1:342,1:totwidth+1,1)=Blue;
 
 tr=zeros(firstt,totwidth,1,"uint8"); #same but with green
 
 tr(1:342,1:totwidth+1,1)=Green;
 
 
 windowsize=130; #window size is 120x120
 startingpoint= 171; #estimated center point of blue channel image, this just centeres the template window
 
 window1=tr(startingpoint-(windowsize/2):startingpoint+(windowsize/2), startingpoint-(windowsize/2):startingpoint+(windowsize/2)); 
 #^ creates template window in green image
  x=0;# sets y and x to 0 just in case
  y=0;
  ssdholder=100000000;  #sets ssd holder very high just in case, lowest ssd is best
  gx=0;
  gy=0;
  for y = 80:260 #iterates from 80 to 260 in both y and x in loop, essentially creating a path for the template to move along
    for x = 80:260
      window2=te(x-(windowsize/2):x+(windowsize/2), y-(windowsize/2):y+(windowsize/2)); #creates window of blue image to match with green
      diffsquared=sum((double(window2(:))-double(window1(:))).^2); #calculates ssd
      #printf("%d", diffsquared);
      #printf(" ");
  
      
      if(diffsquared<ssdholder) #looking for lowest ssd value, so if current value is lower then saves its location
        ssdholder=diffsquared;
        gx=x;
        gy=y;
        
      endif
      x+=1;
    endfor
    
    y+=1;
  endfor
  #printf("%d",totwidth); 
  
  
  tr=zeros(firstt,totwidth,1,"uint8");
 
 tg(1:342,1:totwidth+1,1)=Red; #repeats ssd alforithm but with the red image instead of green
 
 

 
 window3=tg(startingpoint-(windowsize/2):startingpoint+(windowsize/2), startingpoint-(windowsize/2):startingpoint+(windowsize/2));
 
  x=0;
  y=0;
  ssdholder=100000000;
  rx=0;
  ry=0;
  for y = 80:260
    for x = 80:260
      window2=te(x-(windowsize/2):x+(windowsize/2), y-(windowsize/2):y+(windowsize/2));
      diffsquared=sum((double(window2(:))-double(window3(:))).^2);
      #printf("%d", diffsquared);
      #printf(" ");
  
      
      if(diffsquared<ssdholder)
        ssdholder=diffsquared;
        rx=x;
        ry=y;
        
      endif
      x+=1;
    endfor
    
    y+=1;
  endfor
 

 

 
 
 gx=gx-171; #calculates adjustment based on where the original image was based from
 gy=gy-171;
 rx=rx-171;
 ry=ry-171;
 
 
 
 final=zeros(firstt+200,totwidth+200,2,"uint8"); #creates final matrix just like in prev function
 final(101:442,101:totwidth+101,3)=Blue;
 final(101+gx:442+gx,101+gy:totwidth+101+gy,2)=Green;
 final(101+rx:442+rx,101+ry:totwidth+101+ry,1)=Red;
 
 x=gx;
 y=gy;
 
 image=image(1:end-4);

 image = strcat(image, "-ssd.jpg"); #adds ssd to image file name
 imwrite(final,image);

 endfunction
 
 
 
 
 
 function [x,y]=im_align2(image) #essentially the same as align 1 until the ssd/ncc calculation

 info=imfinfo(image); #same as in function before
 totheight=info.Height;
 totwidth=info.Width;
 
 firstt=341;
 secondt=682;
 thirdt=1023;
 
  Blue = imread(image,"PixelRegion", {[0 firstt], [0 totwidth]});
 Green = imread(image,"PixelRegion", {[firstt secondt], [0 totwidth]});
 Red = imread(image,"PixelRegion", {[secondt thirdt], [0 totwidth]});
 
 te=zeros(firstt,totwidth,1,"uint8");
 te(1:342,1:totwidth+1,1)=Blue;
 
 tr=zeros(firstt,totwidth,1,"uint8");
 
 tr(1:342,1:totwidth+1,1)=Green;
 
 
 windowsize=130;
 startingpoint= 171; 
 
 window1=tr(startingpoint-(windowsize/2):startingpoint+(windowsize/2), startingpoint-(windowsize/2):startingpoint+(windowsize/2));
 #^creates template from green image
  x=0;
  y=0;
  nccholder=-1;
  gx=0;
  gy=0;
  ncc=0;
 
      
  ncc= normxcorr2(window1,te); #calculates ncc of template on entire blue image then stores in matrix
      #printf("%d", diffsquared);
      #printf(" ");
  
  [gx,gy]=find(ncc==max(ncc(:))); #find max ncc value and stores its x and y
    
    
  
  
 tr=zeros(firstt,totwidth,1,"uint8"); #repeats the same as above but for red channel
 
 tg(1:342,1:totwidth+1,1)=Red;
 
 

 
 window3=tg(startingpoint-(windowsize/2):startingpoint+(windowsize/2), startingpoint-(windowsize/2):startingpoint+(windowsize/2));
 
  x=0;
  y=0;
  nccholder=-1;
  rx=0;
  ry=0;
  
 
  ncc=normxcorr2(window3, te);
      #printf("%d", diffsquared);
      #printf(" ");
  
  [rx,ry]=find(ncc==max(ncc(:)));
        
     
    
 

 

 #printf("%d", size(window1,1));
 #printf("%d", gy);
 
 
 gx=gx-238; #value that is needed to be changed to translation to align with padding
 gy=gy-238;
 rx=rx-238;
 ry=ry-238;
 
 
 final=zeros(firstt+200,totwidth+200,2,"uint8"); #combines all channels but has padding just in case 
 final(101:442,101:totwidth+101,3)=Blue;
 final(101+gx:442+gx,101+gy:totwidth+101+gy,2)=Green;
 final(101+rx:442+rx,101+ry:totwidth+101+ry,1)=Red;
 
 x=gx;
 y=gy;
 
 image=image(1:end-4);

 image = strcat(image, "-ncc.jpg");
 imwrite(final,image); #writes to file
 
 
 
 
 
 

 endfunction