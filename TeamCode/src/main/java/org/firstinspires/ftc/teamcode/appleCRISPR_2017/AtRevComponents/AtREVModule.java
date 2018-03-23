package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


/**
 * Manages the robot components. 'At' stands for Apple Tau!
 *
 * Although the name refers to a singular REV expansion hub, this object can (and should) contain all components anyways.
 * The name "AtRevController" is a better one, but I haven't bothered to refactor the name.
 */
public class AtREVModule {
    //Important list of components
    private ArrayList<AtREVComponent> components = new ArrayList<>();

    /**
     * Initialize the components.
     *
     * @param hardwareMap The map of attached hardware devices.
     * @return True if all components are found, False otherwise.
     */
    public boolean initialize(HardwareMap hardwareMap) {
        boolean success = true;
        for (int ii = 0; ii < components.size(); ii++) {
            success &= components.get(ii).init(hardwareMap);
        }
        return success;
    }

    /**
     * Convenient all-stop function in case of emergency
     *
     * calls stop() to every component in list
     */
    public void allStop() {
        for (AtREVComponent component : components) {
            component.stop();
        }
    }

    /**
     * Get a component from the list of components
     *
     * @param name Name of the component, as it matches to the config on robot controller phone
     * @return the component you searhed for. Returns null if name doesn't match the list
     */
    public AtREVComponent get(String name){
        for (int ii = 0; ii < components.size(); ii++){
            if (components.get(ii).name.equals(name)){
                return components.get(ii);
            }
        }
        return null;
    }

    /**
     * Adds a component to the list of components
     *
     * @param component component you want to add
     * @return the component you added. If you store the returned AtREVComponent as a local field ('variable') in your OpMode object,
     *      you don't have to use get() to retrieve the object later!
     */
    public AtREVComponent add(AtREVComponent component){
        components.add(component);
        return component;
    }
}
