# Shuffleboard Configuration - ADVKIT Branch
## Important Instructions

**Before you proceed, ensure you are working on the `ADVKIT` branch.** This configuration is specifically tailored for development and testing purposes.

### Using the Included Configuration (.json)

For development and testing on your **development robot**, you can utilize the provided `.json` file to quickly set up your Shuffleboard layout.

**Steps:**

1.  **Locate the `.json` file:** Find the `shuffleboard_layout.json` (or similar) file within this repository.
2.  **Open Shuffleboard:** Launch your Shuffleboard application.
3.  **Import the layout:**
    * Go to `File` > `Open Layout...` (or similar, depending on your Shuffleboard version).
    * Select the `.json` file from this repository.
4.  **Verify the layout:** Ensure all your desired widgets and tabs are correctly loaded.

### Competition Robot Configuration

**DO NOT USE THE INCLUDED `.json` FILE ON YOUR COMPETITION ROBOT.**

For your competition robot, it is crucial to allow Shuffleboard to automatically populate its layout during runtime. This ensures that all necessary data is captured and displayed accurately.

**Steps:**

1.  **Start with a clean slate:** Do not import any pre-configured layouts ONLY USE THE JSON WITH THE TESTING ROBOT!!!
2.  **Run your robot code:** Deploy and run your robot code on the competition robot.
3.  **Open Shuffleboard:** Launch Shuffleboard.
4.  **Observe automatic population:** Shuffleboard will automatically detect and display the data sent from your robot code.

**Note:** Always refer to the latest Shuffleboard documentation for specific instructions and features.
