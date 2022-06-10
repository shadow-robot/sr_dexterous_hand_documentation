%% Class is based on the default book class and options will be passed
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{shadow-manual}[03-06-2022 v0.0.1 Shadow Robot Manual]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ProcessOptions\relax
\LoadClass[10pt,oneside]{book}

%% Main packages in the document --- Some are imported later in the class file
\RequirePackage{mathtools}  % Mathematical tools to use with amsmath
\RequirePackage{amssymb}    % Extended symbol collection
\RequirePackage{siunitx}    % Comprehensive (SI) units package

\RequirePackage{tabularx}   % Tabulars with adjustable-width columns
\RequirePackage{booktabs}   % Publication quality tables
\RequirePackage{longtable}  % Allow tables to flow over page boundaries
\RequirePackage{multirow}   % Create tabular cells spanning multiple rows

\RequirePackage{graphicx}   % Enhanced support for images
\RequirePackage{float}      % Improved interface for floating objects
\RequirePackage[labelfont=bf,justification=centering,footnotesize]{caption} % Captions
\RequirePackage{subcaption} % Support for sub-captions
\RequirePackage{pdfpages}   % Include PDF documents

\RequirePackage[pdfusetitle,hidelinks]{hyperref} % Extensive support for hypertext
\RequirePackage[noabbrev]{cleveref} % Intelligent cross-referencing
\RequirePackage{xcolor}     % Driver-independent color extensions
\RequirePackage{tikz}       % Create PostScript and PDF graphics
\RequirePackage{xspace}     % Define commands that appear not to eat spaces
\RequirePackage{microtype}  % Refinements towards typographical perfection

\RequirePackage{geometry}   % Customize document dimensions
\RequirePackage{titlesec}   % Select alternative section titles
\RequirePackage{titletoc}   % Alternative headings for toc
\RequirePackage{fancyhdr}   % Control of page headers and footers
\RequirePackage{enumitem}   % Control layout of itemize, enumerate, description
\RequirePackage{etoolbox}   % Toolbox of programming facilities
\RequirePackage{iftex}      % Adds if-else statements to support multiple compilers
\RequirePackage{datetime}   % Change format of \today

%% Defining commands for the cover image, subtitle, subject, copyright and table of authors
\newcommand*\subtitle[1]{\def\@subtitle{#1}}
\newcommand*\subject[1]{\def\@subject{#1}}
\newcommand*\shadowcopyright[1]{\def\@shadowcopyright{#1}}
\newcommand*\shadowdateofmanual[1]{\def\@shadowdateofmanual{#1}}
\newcommand*\shadowrelease[1]{\def\@shadowrelease{#1}}
\newcommand*\coverimage[1]{\def\@coverimage{#1}}
\newcommand*\covertable[1]{\def\@covertable{#1}}

%% Scaling the margins to be slightly smaller than default (.7)
\geometry{a4paper,hscale=0.75,vscale=0.8}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% I. Loading the fonts %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\ifPDFTeX
    %% pdfLaTeX is available for compatibility and uses a Paletino-like font
    \RequirePackage[T1]{fontenc}  % Selecting T1 font encoding
    \RequirePackage{newpxtext}    % Palatino-like font...
    \RequirePackage{newpxmath}    % ...with support for mathematics

    %% Another font can be used as the regular and large title style font if desired
    \def\titlestyle{\normalfont}
    \def\largetitlestyle{\normalfont}

\else
    \RequirePackage{fontspec} % Advanced font selection

    %\def\titlestyle{\normalfont} % Customize the regular title style font
    \newfontfamily\largetitlestyle[Path=]{Roboto-Medium.ttf}
    \newfontfamily\robotostyle[Path=]{Roboto-Regular.ttf}
    \newfontfamily\titlestyle[Path=]{Roboto-Regular.ttf}

    \setmainfont{Roboto-Regular}
    %% Changing the quote environment to use Georgia
    %\AtBeginEnvironment{quote}{\quotefont}
\fi

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% II. Adjusting the layout of the titles %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Formatting chapter titles and spacing
\titleformat{\chapter}[display]
    {\flushright}
    {\fontsize{96}{96}\selectfont\largetitlestyle\thechapter}
    {0pt}
    {\Huge\titlestyle}
\titlespacing*{\chapter}{0pt}{0pt}{2\baselineskip}

%% Formatting section titles and spacing
\titleformat{\section}
    {\Large\titlestyle\bfseries}
    {\thesection.}
    {5pt}
    {}
\titlespacing*{\section}{0pt}{\baselineskip}{0pt}

%% Formatting subsections titles and spacing
\titleformat{\subsection}
    {\large\titlestyle\bfseries}
    {\thesubsection.}
    {5pt}
    {}
\titlespacing*{\subsection}{0pt}{\baselineskip}{0pt}

%% Formatting subsubsections titles and spacing
\titleformat{\subsubsection}
    {\titlestyle\bfseries}
    {}
    {0pt}
    {}
\titlespacing*{\subsubsection}{0pt}{\bigskipamount}{0pt}

%% Reducing the vertical white space between chapters in the table of contents
\dottedcontents{chapter}[1.5em]{\vspace{0.5\baselineskip}\bfseries}{1.5em}{0pt}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% III. Adjusting the header and footer %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Changing the header and footer of 'plain' pages
\fancypagestyle{plain}{%
    \fancyhf{}
    \renewcommand{\headrulewidth}{0pt}
    \fancyfoot[C]{\titlestyle\thepage}}

%% Changing the header and footer of 'fancy' pages (based on twoside option)
\if@twoside
    \fancyhf{}
    \fancyhead[LE,RO]{\titlestyle\thepage}
    \fancyhead[RE]{\titlestyle\nouppercase{\leftmark}}
    \fancyhead[LO]{\titlestyle\nouppercase{\rightmark}}
    \RequirePackage{emptypage} % Remove header and footer on empty pages
\else
    \fancyhf{}
    \fancyhead[R]{\titlestyle\thepage}
    \fancyhead[L]{\titlestyle\nouppercase{\rightmark}}
\fi

\pagestyle{fancy} % Setting the package defaults and the additional changes as the style

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% IV. Creating the cover page %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\newcommand*\makecover{
    %% Use the Tikz library positioning and clear the page header and footer
    \usetikzlibrary{positioning}
    \thispagestyle{empty}

    %% Constructing the cover page (cover image, name of university, logo and title text respectively)
    \begin{tikzpicture}[overlay,remember picture]

    \node[above=0,inner sep=0] at (current page.south) {\includegraphics[width=\paperwidth]{\@coverimage}};

    \node[scale=.6,below left=10mm and 20mm] at (current page.north east) {%
        \includegraphics[width=0.35\linewidth]{logo-pink.png}};

    \node[below=6cm,fill=black!85,minimum width={\paperwidth},inner ysep=25pt,opacity=0.0,text opacity=1] at (current page.north) {%
        \begin{minipage}{0.8\paperwidth}
            \largetitlestyle\fontsize{30}{50}\selectfont\color{title}\@title \\[0.5ex]
            \color{black}\titlestyle\fontsize{22}{22}\selectfont%
            \largetitlestyle\ifdefvoid{\@subtitle}{}{\@subtitle \\[2.5ex]}%
            \ifdefvoid{\@subject}{}{\@subject \\[0.5ex]}
        \end{minipage}};

    \node[above,fill=white!85,minimum width={\paperwidth},inner ysep=25pt,opacity=0.6,text opacity=1] at (current page.south) {%
        \begin{minipage}{0.8\paperwidth}
            \color{black}\titlestyle\fontsize{14}{14}\selectfont%
            \begin{center}
            Date of issue:
            \largetitlestyle{\@shadowdateofmanual}
            \hspace{25mm}
            \titlestyle{Release Version:}
            \largetitlestyle{\@shadowrelease \\[2.5ex]}
            \end{center}
        \end{minipage}};
    \node[xshift=4mm,yshift=3mm,anchor=south] at (current page.south){%
        \includegraphics[scale=0.9]{line.png}};

    \end{tikzpicture}
    \newpage
    \begin{tikzpicture}[overlay,remember picture]
    \node[rotate=90,below right=40mm and 3mm] at (current page.west) {\ifdefvoid{\@shadowcopyright}{}{\color{black}\titlestyle\@shadowcopyright}};
    \end{tikzpicture}

    \noindent\fbox{%
        \parbox{\textwidth}{%
            \titlestyle{
            The information contained herein is the property of Shadow Robot Company and shall not be
            reproduced in whole or in part without prior written approval of Shadow Robot Company. The
            information herein is subject to change without notice and should not be construed as a
            commitment by Shadow Robot Company. This document is periodically reviewed and revised.

            \vspace*{5mm}
            Shadow Robot Company assumes no responsibility for any errors or omissions in this document.
            \vspace*{5mm}

            Copyright © 2018-2022 by Shadow Robot Company.
            \vspace*{5mm}

            Shadow\textregistered \hspace{1mm}  is a registered trademark of The Shadow Robot Company Ltd}
        }%
    }
    \newpage
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% V. Setting up listings %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\RequirePackage{listings} % Typeset source code listings

\lstset{
  basicstyle=\ttfamily\footnotesize,  % Style of the font that is used for the code
  backgroundcolor=\color{gray!10},    % Background color
  keywordstyle=\color{red!75!black},  % Keyword style
  stringstyle=\color{green!40!black}, % String style
  commentstyle=\color{blue!30!black}, % Comment style
  numbers=left,                       % Add line numbers on the left side
  numbersep=5pt,                      % Decrease distance between line numbers and code
  numberstyle=\tiny,                  % Line number style
  breaklines=true,                    % Line break automatically
}
