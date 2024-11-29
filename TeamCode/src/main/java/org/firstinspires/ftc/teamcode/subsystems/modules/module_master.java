package org.firstinspires.ftc.teamcode.subsystems.modules;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.CLAW_OPEN;

import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.test.dummyServo;

/* contains sample action could be done in auto. Dalshe ebites sami */
public class module_master {
    /* add actions here following byte flags conception */
    public enum action{;
        public static final int CLAW_CLOSE = 0b0000001;
        public static final int CLAW_OPEN = 0b0000010;
    }

    static differential differential;
    static arm arm;

    public static void init(){

    }
    public static boolean doAction(int action){
        /* continue ifing this as shown */
        if ((action & CLAW_CLOSE) == CLAW_CLOSE){
            action -= CLAW_CLOSE;
            /* call function that closes claw */
            differential.closeClaw();
        }

        if ((action & CLAW_OPEN) == CLAW_OPEN){
            action -= CLAW_OPEN;
            /* call function that opens claw */
            differential.openClaw();
        }

        /* check if all actions are done */
        return action == 0;
    }

    public void update(){
        arm.update();
        differential.update();
    }

}
