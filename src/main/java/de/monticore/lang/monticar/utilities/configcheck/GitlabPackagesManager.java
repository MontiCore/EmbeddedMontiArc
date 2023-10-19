package de.monticore.lang.monticar.utilities.configcheck;

import org.gitlab4j.api.GitLabApi;
import org.gitlab4j.api.GitLabApiException;
import org.gitlab4j.api.PackagesApi;
import org.gitlab4j.api.models.Package;
import org.gitlab4j.api.models.PackageFile;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class GitlabPackagesManager {
    private static final String GITLAB_API_URL = "https://git.rwth-aachen.de";
    private static final String PRIVATE_TOKEN = "glpat-Dr2YZgwdmdPWSehyZtWq";
    private static final String PROJECT_ID = "49355";
    private static final String PACKAGE_NAME = "package-name-to-upload";
    private static final String PACKAGE_VERSION = "1.0.0";
    private GitLabApi gitLabApi;

    public GitlabPackagesManager() {
        gitLabApi = new GitLabApi(GITLAB_API_URL, PRIVATE_TOKEN);
    }

//    public Package searchPackage(){
//        PackagesApi packagesApi = gitLabApi.getPackagesApi();
//        List<Package> packages = null;
//
//        try {
//            packages = packagesApi.getPackages(PROJECT_ID);
//        } catch (GitLabApiException e) {
//            e.printStackTrace();
//        }
//
//        for (Package pkg : packages) {
//            if (PACKAGE_NAME.equals(pkg.getName())) {
//                return pkg;
//            }
//        }
//
//        return null;
//    }

    public List<Package> getPackages(Object projectIdOrPath) {
        PackagesApi packagesApi = gitLabApi.getPackagesApi();
        List<Package> packages = new ArrayList<>();
        try {
            packages = packagesApi.getPackages(projectIdOrPath);
        } catch (GitLabApiException e) {
            e.printStackTrace();
        }
        return packages;
    }

    public void downloadPackage(Package foundPackage) {
        // TODO: not a real download! Just search through the API
//        PackagesApi packagesApi = gitLabApi.getPackagesApi();
//
//        try {
//            List<PackageFile> packageFiles = packagesApi.getPackageFiles(PROJECT_ID, foundPackage.getId());
//            if (!packageFiles.isEmpty()) {
//                PackageFile packageFile = packageFiles.get(0); // You can choose the desired package file
//                File downloadPath = new File("downloaded-package.tar.gz"); // Specify your download path
//                packagesApi.downloadPackageFile(packageFile.getId(), downloadPath);
//                System.out.println("Package downloaded to " + downloadPath.getAbsolutePath());
//            } else {
//                System.out.println("No package files found.");
//            }
//        } catch (GitLabApiException e) {
//            e.printStackTrace();
//        }
    }

//    public void uploadPackage(File packageFile) { // Replace "YOUR_PACKAGE_PATH" with the actual path to your package file
//        PackagesApi packagesApi = gitLabApi.getPackagesApi();
//        try {
//            PackageFileUpload packageFileUpload = new PackageFileUpload()
//                    .withPackageName(PACKAGE_NAME)
//                    .withVersion(PACKAGE_VERSION)
//                    .withFile(packageFile);
//
//            PackageFile uploadedPackageFile = packagesApi.createPackageFile(PROJECT_ID, packageFileUpload);
//            System.out.println("Package uploaded with ID: " + uploadedPackageFile.getId());
//        } catch (GitLabApiException e) {
//            e.printStackTrace();
//        }
//    }

    public static void main(String[] args) {
        GitlabPackagesManager packageHandler = new GitlabPackagesManager();
        List<Package> packages = packageHandler.getPackages(PROJECT_ID);

        if (packages.size() == 0) {
            System.out.println("No packages found for project_id: " + PROJECT_ID);
        }

        for (Package p : packages) {
            System.out.println(p.getName());
            System.out.println(p.getVersion());
            System.out.println(p.getId());
            System.out.println(p.getPackageType());
        }
    }
}
