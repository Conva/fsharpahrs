#!/usr/bin/env bash
dotnet test /p:CollectCoverage=true /p:CoverletOutputFormat=opencover &&
dotnet reportgenerator "-reports:coverage.opencover.xml" "-targetdir:coverage"