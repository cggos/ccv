package ghc.threads;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import javax.swing.JLabel;
import javax.swing.SwingUtilities;

public class TimerThread extends Thread {

	protected boolean isRunning;
	 
    protected JLabel dateLabel;
    protected JLabel timeLabel;

    protected SimpleDateFormat dateFormat = 
            new SimpleDateFormat("yyyy-MM-dd");//EEE, d MMM yyyy
    protected SimpleDateFormat timeFormat =
            new SimpleDateFormat("HH:mm:ss");//h:mm a

    public TimerThread(JLabel dateLabel, JLabel timeLabel) {
        this.dateLabel = dateLabel;
        this.timeLabel = timeLabel;
        this.isRunning = true;
    }

    @Override
    public void run() {
        while (isRunning) {
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    Calendar currentCalendar = 
                        Calendar.getInstance();
                    Date currentTime = 
                        currentCalendar.getTime();
                    dateLabel.setText(dateFormat.format(currentTime));
                    timeLabel.setText(timeFormat.format(currentTime));
                }
            });

            try {
                Thread.sleep(1000L);
            } catch (InterruptedException e) {
            }
        }
    }

    public void setRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }
}
