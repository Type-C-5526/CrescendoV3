package frc.robot.util;

public class CounterTimer implements Runnable, AutoCloseable {

    private double m_counter;


    public CounterTimer(){

    }

    public void reset(){
        m_counter = 0;
    }

    @Override
    public void run(){

    }

    @Override
    public void close(){

    }
    
}
