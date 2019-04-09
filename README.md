# BICA-dialog
**BICA-dialog simplifies the task of developing actions or behaviors related to dialogue.**

BICA-dialog contains the library DialogInterface from which we will inherit to develop our dialogue actions. Each action will be specific to an intent ([Dialogflow concepts](https://dialogflow.com/docs)). The library offers methods to do text-to-speech tasks through [Google Speech sevice](https://cloud.google.com/speech-to-text/) and methods to do Natural Language Processing tasks through [Dialogflow](https://dialogflow.com/), using the ROS package dialogflow_ros ([official package](https://wiki.ros.org/dialogflow_ros),  [our custom dialogflow_ros](https://github.com/jginesclavero/dialogflow_ros)). The library also offers a method to do text-to-speech through the package [sound_play](https://wiki.ros.org/sound_play).

## Installing
If you don't have vcs tool, install it with ```sudo apt-get install python3-vcstool ```
Import dependencies with ``` vcs import < bica_dialog_requirements.repos ```
Install python requirements of dialogflow_ros [README](https://github.com/jginesclavero/dialogflow_ros/tree/master/dialogflow_ros))

### Creating a virtualenv for install Python modules
If you have dependencies issues when you installed the above requirements
Create a virtualenv ```virtualenv venv --system-site-packages```
Activate the virtual env ``` source venv/bin/activate ```
Install the dependencies ``` pip install -r [__BICA_Dialog_Path__]/dialogflow_ros/dialogflow_ros/requirements.txt ```

**Remind activate the virtualenv in each shell where you want use dialogflow_ros**

## Use

Below is an example of using the BICA-dialog library ([example file](https://github.com/IntelligentRoboticsLabs/BICA-dialog/blob/master/bica_dialog/test/test_DialogInterface.cpp)).
First of all we define our new class that inherits from DialogInterface, passing the intent for which our action was defined.
We have to redefine the listenCallback method to read the Dialogflow output like a ROS message.

```
class DialogInterfaceTest: public bica_dialog::DialogInterface
{
  public:
    std::string intent_;
    explicit DialogInterfaceTest(std::string intent): DialogInterface(intent){}
    void listenCallback(dialogflow_ros::DialogflowResult result)
    {
      ROS_INFO("[DialogInterfaceTest] listenCallback: intent %s", result.intent.c_str());
      intent_ = result.intent;
    }
};

```
When we have our class instantiated we can use the methods *speak* or *listen*. Both methods are **syncronous**

```
bica_dialog::DialogInterfaceTest di(intent_in);
di.speak("Hello world!")
```
```
bica_dialog::DialogInterfaceTest di(intent_in);
di.listen();
```

## Tests
Compile ``` catkin_make tests bica_dialog ``` and execute the test 

``` 
roslaunch bica_dialog test_DialogInterface.launch
rosrun bica_dialog bica_dialog-test

```
