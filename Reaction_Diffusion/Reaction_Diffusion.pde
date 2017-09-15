int H = 480;
int W = 270;
PImage img = createImage(H, W, RGB);

//sliders
import controlP5.*;
ControlP5 cp5;

//System parameters
double diffU;
double diffV;
double paramF;
double paramK;

boolean rndInitCondition;

double[][] U = new double[H][W];
double[][] V = new double[H][W];

double[][] dU = new double[H][W];
double[][] dV = new double[H][W];

int[][] horizOffset = new int[W][2];
int[][] vertOffset = new int[H][2];


void generateInitialState() {
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) { 
      U[i][j] = 1.0;
      V[i][j] = 0.0;
    }
  }

  if (rndInitCondition) {
    for (int i = 0; i < H; i++) {
      for (int j = 0; j < W; j++) {     
        if (pow(i-H/2, 2) + pow(j-W/2, 2) < 400) {  
          if (random(0, 1) > .8) {
            U[i][j] = random(0, 1);
            V[i][j] = random(0, 1);
          }
        }
      }
    }
  } else {
    for (int i = 0; i < H; i++) {
      for (int j = 0; j < W; j++) {     
        if (pow(i-H/2, 2) + pow(j-W/2, 2) < 400) {
          U[i][j] = 0;
          V[i][j] = 1;
        }
      }
    }
  }
}

void setup() {
  //fullScreen(P2D);
  size(960, 540, P2D);
  frame.setResizable(true);
  surface.setResizable(true);
  frameRate(30);
  smooth();
  colorMode(HSB, 1.0);


  //Set default parameters;
  diffU = 0.16;
  diffV = 0.08;
  paramF = 0.035;
  paramK = 0.06;

  //sliders
  cp5 = new ControlP5(this);
  
  cp5.addTextlabel("label1")
                    .setText("[n] new")
                    .setPosition(600,10)
                    .setFont(createFont("Georgia",20))
                    ;
  cp5.addTextlabel("label2")
                    .setText("[r] random")
                    .setPosition(600,50)
                    .setFont(createFont("Georgia",20))
                    ;
  cp5.addTextlabel("label3")
                    .setText("[1-8] presets")
                    .setPosition(800,10)
                    .setFont(createFont("Georgia",20))
                    ;
  
  cp5.addSlider("Feed")
    .setPosition(10, 10)
    .setSize(400, 30)
    .setRange(.01, .1)
    .setValue((float)paramF)
    .setDecimalPrecision(4)
    .setFont(createFont("Ubuntu", 20, true))
    ;
  cp5.addSlider("Kill")
    .setPosition(10, 50)
    .setSize(400, 30)
    .setRange(.045, .07)
    .setValue((float)paramK)
    .setDecimalPrecision(4)
    .setFont(createFont("Ubuntu", 20, true))
    ;

  rndInitCondition = true;

  //Populate U and V with initial data
  generateInitialState();

  //Set up offsets 
  for (int i = 1; i < W-1; i++) {
    horizOffset[i][0] = i-1;
    horizOffset[i][1] = i+1;
  }

  horizOffset[0][0] = W-1;
  horizOffset[0][1] = 1;

  horizOffset[W-1][0] = W-2;
  horizOffset[W-1][1] = 0;

  for (int i = 1; i < H-1; i++) {
    vertOffset[i][0] = i-1;
    vertOffset[i][1] = i+1;
  }

  vertOffset[0][0] = H-1;
  horizOffset[0][1] = 1;

  vertOffset[H-1][0] = H-2;
  vertOffset[H-1][1] = 0;
}

void timestep(double F, double K, double diffU, double diffV) {
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {

      double u = U[i][j];
      double v = V[i][j];

      int up = vertOffset[i][0];
      int down = vertOffset[i][1];
      int left = horizOffset[j][0];
      int right = horizOffset[j][1];

      double uvv = u*v*v;     
      double lapU = (U[up][j] + U[down][j] + U[i][left] + U[i][right] - 4*u);
      double lapV = (V[up][j] + V[down][j] + V[i][left] + V[i][right] - 4*v);

      dU[i][j] = diffU*lapU  - uvv + F*(1 - u);
      dV[i][j] = diffV*lapV + uvv - (K+F)*v;
    }
  }


  for (int i= 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      U[i][j] += dU[i][j];
      V[i][j] += dV[i][j];
    }
  }
}

void draw() { 
  for (int k = 0; k < 10; k++) {
    timestep(paramF, paramK, diffU, diffV);
  }

  img.loadPixels();
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {

      float val = (float)(1-U[i][j]);
      //purple
      //img.pixels[i*W + j] = color(0.74, 0.87, val); 

      //blueish heatmap
      img.pixels[i + j*H] = color(val, val, val);

      //gray  
      //img.pixels[i*W + j] = color(1-val);
    }
  }
  img.updatePixels();
  image(img, 0, 0, width, height);
  //make black
  //filter(THRESHOLD);
  //cool contrasty filter
  //filter(POSTERIZE, 4);
}


void keyPressed() {
  switch (key) {
  case '1':
    paramF = 0.035;
    paramK = 0.06;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '2':
    paramF = 0.042;
    paramK = 0.065;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '3':
    paramF = 0.025;
    paramK = 0.056;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '4':
    paramF = 0.02;
    paramK = 0.056;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '5':
    paramF = 0.035;
    paramK = 0.065;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '6':
    paramF = 0.062;
    paramK = 0.062;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '7':
    paramF = 0.05;
    paramK = 0.065;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case '8':
    paramF = .0135;
    paramK = 0.047;
    cp5.getController("Feed").setValue((float)paramF);
    cp5.getController("Kill").setValue((float)paramK);
    break;
  case 'r':
    rndInitCondition = true;
    generateInitialState();
    break;
  case 'n':
    rndInitCondition = false;
    generateInitialState();
  }
}

void Feed(float sliderF) {
  paramF = sliderF;
}
void Kill(float sliderK) {
  paramK = sliderK;
}