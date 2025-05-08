// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDConstants;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;
import frc.robot.trobot5013lib.led.IntensityPattern;
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ScannerPattern;
import frc.robot.trobot5013lib.led.AlternatingColorPattern;

public class LED extends SubsystemBase {

	private TrobotAddressableLED m_led = new TrobotAddressableLED(LEDConstants.LED_PWM_PORT, LEDConstants.LED_LENGTH);
	private boolean noteIndicator = false;
	private int intensityDegrees = 1;
    private Color[] redWhiteGreenArray = {Color.kRed, Color.kBlack, Color.kGreen};
    private Color[] blueWhiteArray = {Color.kBlue, Color.kBlack};
	private Color[] pinkOrangeArray = {LEDConstants.pink, LEDConstants.orange};
	private Color[] blackOrangeArray = {Color.kBlack, LEDConstants.orange};
	private Color[] blueOrangeArray = {Color.kBlue, LEDConstants.orange};

	private Color[] redWhiteBlueArray = {Color.kRed, Color.kWhite, Color.kBlue};

	private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);

	private TrobotAddressableLEDPattern m_alternatingColorPattern = new AlternatingColorPattern(redWhiteBlueArray);
	private TrobotAddressableLEDPattern m_blinkingRed = new BlinkingPattern(Color.kRed, 0.25);
	private TrobotAddressableLEDPattern m_blinkingGreen = new BlinkingPattern(Color.kGreen, 0.25);
	private TrobotAddressableLEDPattern m_chaosPattern = new ChaosPattern();
    private TrobotAddressableLEDPattern m_redChasePattern = new ChasePattern(redWhiteGreenArray, 3);
    private TrobotAddressableLEDPattern m_blueChasePattern = new ChasePattern(blueWhiteArray, 3);
	private TrobotAddressableLEDPattern m_orangeChasePattern = new ChasePattern(blackOrangeArray, 6);
	private TrobotAddressableLEDPattern m_redWhiteBlueChasePattern = new ChasePattern(redWhiteBlueArray, 2);
	private IntensityPattern m_blueIntensityPattern = new IntensityPattern(Color.kBlue, intensityDegrees);
	private IntensityPattern m_redIntensityPattern = new IntensityPattern(Color.kRed, intensityDegrees);
	private IntensityPattern m_greenIntensityPattern = new IntensityPattern(Color.kGreen, intensityDegrees);
	private TrobotAddressableLEDPattern m_rainbowPattern = new RainbowPattern();
	private TrobotAddressableLEDPattern m_scannerPattern = new ScannerPattern(Color.kChartreuse,Color.kDeepPink,8);
	private TrobotAddressableLEDPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kLightYellow);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	
	private TrobotAddressableLEDPattern m_active_pattern = m_rainbowPattern;
	
	/** Creates a new StatusLED. */
	public LED() {
		super();
	}

	@Override
	public void periodic() {
		//m_led.setPattern(m_active_pattern);
	}

	public void start(){
		m_led.setPattern(m_active_pattern);
		System.out.println("led start");
	}
}
