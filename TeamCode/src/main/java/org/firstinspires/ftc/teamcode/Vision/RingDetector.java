package org.firstinspires.ftc.teamcode.Vision;

// Vision Hardware
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class RingDetector {

   private Vuforia vuforia;
   private ObjectDetector od;

   public enum Config {
      NONE,
      SINGLE,
      QUAD
   }

   public RingDetector(){
      vuforia = new Vuforia();
      ObjectDetector od = new ObjectDetector();
   }

   public Config getRingConfig(){

      Recognition recog = od.getPrimaryObject();
      if (recog == null) throw new Error("No object detected");

      switch (recog.getLabel()){

         case "Single":
            return Config.SINGLE;
         case "Quad":
             return Config.QUAD;
      }
     return Config.NONE;
   }
}
