# Smart_lighting
پروپزال پروژه روشنایی هوشمند مبتنی بر سیستم‌های نهفته و بیدرنگ

عنوان پروژه
طراحی و پیاده‌سازی یک روشنایی هوشمند با قابلیت‌های تشخیص صدا، نور و حرکت مبتنی بر رزبری پای پیکو (با قابلیت جایگزینی اختیاری با ESP32)

مقدمه
با پیشرفت فناوری‌های اینترنت اشیا (IoT) و سیستم‌های نهفته، نیاز به سیستم‌های هوشمند و خودکار در زندگی روزمره بیش از پیش احساس می‌شود. روشنایی‌های هوشمند به عنوان یکی از کاربردهای اصلی این فناوری‌ها، می‌توانند با استفاده از سنسورهای مختلف، تجربه‌ای بهتر و کارآمدتر را برای کاربران فراهم کنند. این پروژه به طراحی و پیاده‌سازی یک روشنایی هوشمند می‌پردازد که با استفاده از سنسورهای نور، تشخیص صدا و حرکت، قابلیت‌های متنوعی را ارائه می‌دهد. در این پروژه، از رزبری پای پیکو به عنوان پردازنده اصلی استفاده می‌شود، اما در صورت نیاز، امکان جایگزینی آن با ESP32 نیز به عنوان گزینه اختیاری در نظر گرفته شده است.

اهداف پروژه
هدف اصلی:
طراحی و پیاده‌سازی یک روشنایی هوشمند که با استفاده از سنسورهای نور، تشخیص صدا و حرکت، بتواند به صورت خودکار و هوشمند روشن و خاموش شود.

اهداف فرعی:

استفاده از میکروفون خازنی برای تشخیص صدا و کنترل روشنایی با دست زدن (دو بار برای روشن کردن و سه بار برای خاموش کردن).

استفاده از سنسور نور برای تنظیم حساسیت روشنایی در شرایط نوری مختلف.

استفاده از سنسور حرکت برای خاموش کردن خودکار روشنایی در صورت عدم حرکت در بازه‌های زمانی مشخص.

افزودن قابلیت‌های اختیاری مانند کنترل روشنایی از طریق وای‌فای و اپلیکیشن موبایل.

امکان استفاده از رزبری پای پیکو به عنوان پردازنده اصلی و ESP32 به عنوان گزینه جایگزین اختیاری.

شرح پروژه
بخش‌های اصلی پروژه
میکروفون خازنی و تشخیص صدا:

استفاده از میکروفون خازنی برای تشخیص صدا و تشخیص تعداد دست زدن‌ها.

دو بار دست زدن برای روشن کردن روشنایی و سه بار دست زدن برای خاموش کردن آن.

سنسور نور:

استفاده از سنسور نور برای تشخیص سطح نور محیط.

اگر نور محیط از حد مشخصی کمتر باشد، روشنایی با دو بار دست زدن روشن و با سه بار دست زدن خاموش می‌شود.

اگر نور محیط از حد مشخصی بیشتر باشد، روشنایی تنها با چهار بار دست زدن روشن یا خاموش می‌شود.

سنسور حرکت:

استفاده از سنسور حرکت برای تشخیص حرکت‌های بزرگ در محیط.

اگر در طول 10 دقیقه حرکت بزرگی تشخیص داده نشود، روشنایی به صورت خودکار خاموش می‌شود.

بخش‌های اختیاری پروژه
ماژول وای‌فای و کنترل از طریق اپلیکیشن موبایل:

افزودن ماژول وای‌فای به سیستم برای کنترل روشنایی از طریق اپلیکیشن موبایل.

قابلیت تنظیم زمان‌بندی برای روشن و خاموش کردن روشنایی در ساعت‌های خاص.

قابلیت تنظیم آلارم با چشمک زدن روشنایی.

بلندگو و پخش موسیقی:

افزودن بلندگو به سیستم برای پخش موسیقی و آهنگ‌های مختلف.

کنترل پخش موسیقی از طریق اپلیکیشن موبایل.

جایگزینی پردازنده اصلی:

در صورت نیاز، امکان جایگزینی رزبری پای پیکو با ESP32 به عنوان پردازنده اصلی وجود دارد. این گزینه به دلیل قابلیت‌های ارتباطی پیشرفته‌تر ESP32 (مانند پشتیبانی از وای‌فای و بلوتوث) در نظر گرفته شده است.

ابزارها و فناوری‌های مورد نیاز
سخت‌افزار:
پردازنده اصلی: رزبری پای پیکو (با قابلیت جایگزینی اختیاری با ESP32)

میکروفون خازنی

سنسور نور (LDR)

سنسور حرکت (PIR)

ماژول وای‌فای (در صورت استفاده از ESP32، این ماژول به صورت داخلی موجود است)

بلندگو (اختیاری)

LED یا روشنایی قابل کنترل

مقاومت‌ها، خازن‌ها و سایر قطعات الکترونیکی مورد نیاز

نرم‌افزار:
محیط برنامه‌نویسی MicroPython یا C/C++ برای رزبری پای پیکو

محیط برنامه‌نویسی Arduino IDE در صورت استفاده از ESP32

کتابخانه‌های مورد نیاز برای سنسورها و ماژول‌ها

اپلیکیشن موبایل برای کنترل روشنایی (اختیاری)

روش‌شناسی و مراحل اجرای پروژه
تحلیل نیازمندی‌ها:

بررسی نیازهای کاربر و تعیین قابلیت‌های اصلی و اختیاری سیستم.

طراحی سیستم:

طراحی مدار الکترونیکی و انتخاب قطعات مناسب.

طراحی معماری نرم‌افزاری و تعیین وظایف هر بخش.

پیاده‌سازی سخت‌افزاری:

مونتاژ قطعات و تست عملکرد هر بخش.

پیاده‌سازی نرم‌افزاری:

نوشتن کدهای لازم برای کنترل سنسورها و ماژول‌ها.

در صورت استفاده از ESP32، پیاده‌سازی قابلیت‌های ارتباطی مانند وای‌فای و بلوتوث.

تست و بهینه‌سازی:

تست کامل سیستم و رفع اشکالات احتمالی.

بهینه‌سازی کدها و بهبود عملکرد سیستم.

مستند‌سازی:

تهیه گزارش نهایی و مستندات فنی پروژه.

نتیجه‌گیری
این پروژه با ترکیب فناوری‌های مختلف، یک روشنایی هوشمند با قابلیت‌های متنوع طراحی و پیاده‌سازی می‌کند. استفاده از رزبری پای پیکو به عنوان پردازنده اصلی، امکان توسعه‌پذیری و انعطاف‌پذیری بالایی را فراهم می‌کند. همچنین، با در نظر گرفتن ESP32 به عنوان گزینه جایگزین اختیاری، قابلیت‌های ارتباطی پیشرفته‌تری به سیستم اضافه می‌شود. این پروژه می‌تواند به عنوان پایه‌ای برای توسعه سیستم‌های هوشمندتر و پیچیده‌تر در آینده مورد استفاده قرار گیرد.

گروه ارائه‌دهنده:
سهیل صدفی و پویا خاتمی
