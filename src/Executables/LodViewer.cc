#include <filesystem>

#include <QtGui>
#include <QApplication>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include <gflags/gflags.h>

#include <Gui/Window.h>
#include <Geometry/Point.h>
#include <FileIO/BinaryIO.h>
#include <FileIO/OctreeReader.h>

DEFINE_string(octree_file, "", "required");

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::string octree_file = FLAGS_octree_file;

    QApplication app(argc, argv);
    QApplication::setApplicationName("LodViewer");
    QCoreApplication::setApplicationVersion("version 1.0");
    
    // Check if provided files exist. If not display dialog and let
    // user select the files
    QDir path = QDir::current();
    if (!std::filesystem::exists(octree_file)) {
        QMainWindow dialog_window;
        QString tmp_path = QFileDialog::getOpenFileName(
            &dialog_window,
            "Select File",
             path.path(),
             "Octree File (*.octree)",
             nullptr
             );
        dialog_window.show();

        octree_file = tmp_path.toStdString();
        path = QFileInfo(tmp_path).dir();

        if (!std::filesystem::exists(octree_file))
            return 0;
    }

    if (std::filesystem::path(octree_file).extension() != ".octree")
        return 0;
    octree_reader::OctreeReader octree_reader(octree_file);

    gui::Window<double> main_window(octree_reader);
    main_window.show();
    app.exec();

    return 0;
}
