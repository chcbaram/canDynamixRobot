import processing.serial.*;


Serial serial_port = null;        // the serial port


float robot_x;
float robot_y;
float robot_theta;

String  inString;  

final int VIEW_SIZE_X = 600, VIEW_SIZE_Y = 600;



// Serial
//
Button btn_serial_up;              // move up through the serial port list
Button btn_serial_dn;              // move down through the serial port list
Button btn_serial_connect;         // connect to the selected serial port
Button btn_serial_disconnect;      // disconnect from the serial port
Button btn_serial_list_refresh;    // refresh the serial port list
String serial_list;                // list of serial ports
int serial_list_index = 0;         // currently selected serial port 
int num_serial_ports = 0;          // number of serial ports in the list

int btn_x = 240;
int box_x = 4;
int serial_baud = 115200;// serial port buttons





void settings() {
  size(VIEW_SIZE_X, VIEW_SIZE_Y, P3D);
}

void setup() 
{

// create the buttons
  btn_serial_up = new Button("^", btn_x, 10, 40, 20);
  btn_serial_dn = new Button("v", btn_x, 50, 40, 20);
  btn_serial_connect = new Button("Open", btn_x+50, 10, 100, 25);
  btn_serial_list_refresh = new Button("Refresh", btn_x+50, 45, 100, 25);
  
  // get the list of serial ports on the computer
  serial_list = Serial.list()[serial_list_index];
  
  
  // get the number of serial ports in the list
  num_serial_ports = Serial.list().length;
}

void keyPressed() {
  
  if (serial_port != null)
  {
    serial_port.write(key);
  }
}

void mousePressed() {
  // up button clicked
  if (btn_serial_up.MouseIsOver()) {
    if (serial_list_index > 0) {
      // move one position up in the list of serial ports
      serial_list_index--;
      serial_list = Serial.list()[serial_list_index];
    }
  }
  // down button clicked
  if (btn_serial_dn.MouseIsOver()) {
    if (serial_list_index < (num_serial_ports - 1)) {
      // move one position down in the list of serial ports
      serial_list_index++;
      serial_list = Serial.list()[serial_list_index];
    }
  }
  // Connect button clicked
  if (btn_serial_connect.MouseIsOver()) {
    if (serial_port == null) {
      // connect to the selected serial port
      serial_port = new Serial(this, Serial.list()[serial_list_index], serial_baud);
      btn_serial_connect.label = "Close";
      btn_serial_connect.fill_color = 100;
    }
    else
    {
      if (serial_port != null) {
        // disconnect from the serial port
        serial_port.stop();
        serial_port = null;
        btn_serial_connect.label = "Open";
        btn_serial_connect.fill_color = 218;
      }      
    }
  }
  // Refresh button clicked
  if (btn_serial_list_refresh.MouseIsOver()) {
    // get the serial port list and length of the list
    serial_list = Serial.list()[serial_list_index];
    num_serial_ports = Serial.list().length;
  }
}


// function for drawing a text box with title and contents
void DrawTextBox(String title, String str, int x, int y, int w, int h)
{
  fill(255);
  rect(x, y, w, h);
  fill(0);
  textAlign(LEFT);
  textSize(14);
  text(title, x + 10, y + 10, w - 20, 20);
  textSize(12);  
  text(str, x + 10, y + 40, w - 20, h - 10);
}

// button class used for all buttons
class Button {
  String label;
  float x;    // top left corner x position
  float y;    // top left corner y position
  float w;    // width of button
  float h;    // height of button
  int fill_color;
  
  // constructor
  Button(String labelB, float xpos, float ypos, float widthB, float heightB) {
    label = labelB;
    x = xpos;
    y = ypos;
    w = widthB;
    h = heightB;
    
    fill_color = 218;
  }
  
  // draw the button in the window
  void Draw() {
    fill(fill_color);
    stroke(141);
    rect(x, y, w, h, 10);
    textAlign(CENTER, CENTER);
    fill(0);
    text(label, x + (w / 2), y + (h / 2));
  }
  
  // returns true if the mouse cursor is over the button
  boolean MouseIsOver() {
    if (mouseX > x && mouseX < (x + w) && mouseY > y && mouseY < (y + h)) {
      return true;
    }
    return false;
  }
}


void drawSerialList()
{
  // draw the buttons in the application window
  btn_serial_up.Draw();
  btn_serial_dn.Draw();
  btn_serial_connect.Draw();
  btn_serial_list_refresh.Draw();
  // draw the text box containing the selected serial port
  DrawTextBox("Select Port", serial_list, box_x, 10, 220, 60);
}  





void DrawRobot()
{
  int i;

  /*
  fill(0);
  rect( draw_box_x, draw_box_y, draw_box_w, draw_box_h);
  stroke(127,34,255);     //stroke color
  strokeWeight(1);        //stroke wider
  line(line_s_x, line_s_y, line_e_x, line_e_y);       

  stroke(255,0,0);
  strokeWeight(1);  
  line(draw_box_x, box_center_y, draw_box_x+draw_box_w, box_center_y);
  
  for( i=0; i<FBufMax; i++ )
  {
    stroke(FColor[i]);
    Serial_DrawFrame(i);
  } 
  */
  float px;
  float py;
  float radius = 50;
  
  float robot_x_pos;
  float robot_y_pos;
  
  float wl_x;
  float wl_y;
  float wr_x;
  float wr_y;
  
  robot_x_pos = width/2  - robot_y*10;
  robot_y_pos = height/2 - robot_x*10;
  
    
  px = robot_x_pos - cos(radians(robot_theta-90))*(radius);
  py = robot_y_pos + sin(radians(robot_theta-90))*(radius);

  wl_x = robot_x_pos - cos(radians(robot_theta))*(radius/2);
  wl_y = robot_y_pos + sin(radians(robot_theta))*(radius/2);

  wr_x = robot_x_pos - cos(radians(robot_theta+180))*(radius/2);
  wr_y = robot_y_pos + sin(radians(robot_theta+180))*(radius/2);

  
  stroke(255,0,0);
  line(robot_x_pos, robot_y_pos, px, py);
  
  stroke(0,255,0);
  ellipse(robot_x_pos, robot_y_pos, radius, radius);
  
  fill(255);
  stroke(255,255,255);
  ellipse(robot_x_pos, robot_y_pos, 5, 5);
  
  pushMatrix();
  translate(wl_x, wl_y);
  stroke(255);
  rotate(radians(-robot_theta));
  rect(-3, -10, 6, 20);  
  popMatrix();
   
  pushMatrix();
  translate(wr_x, wr_y);
  stroke(255);
  rotate(radians(-robot_theta));
  rect(-3, -10, 6, 20);  
  popMatrix();
  
}


void draw() {  
  
  background(#000000);
  fill(#ffffff);

  stroke(255);
  line(0, height/2, width, height/2);


  drawSerialList();  
  DrawRobot();  
}


void serialEvent(Serial p) {

  inString = (p.readStringUntil('\n'));
  
  try {
  
    if( inString != null )
    { 
      String[] dataStrings = split(inString, ' ');
      robot_x     = float(dataStrings[2]);
      robot_y     = float(dataStrings[5]);
      robot_theta = float(dataStrings[8]);
      
      
      println( robot_x + " " + robot_y + " " + robot_theta );
    }
  } catch (Exception e) {
      println("Caught Exception");
  }
}